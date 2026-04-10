#!/usr/bin/env python3
import argparse
import csv
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from queue import Empty, Full, Queue
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CompressedImage, Image, NavSatFix
from tf2_msgs.msg import TFMessage
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException
from tf_time_domain import (
    derive_lookup_time_offset_sec,
    map_query_time_to_tf_domain as map_query_time_to_tf_domain_impl,
    map_tf_time_to_query_domain as map_tf_time_to_query_domain_impl,
)


IMAGE_TOPIC_TYPES = ("sensor_msgs/CompressedImage", "sensor_msgs/Image")
GPS_TOPIC_TYPES = ("sensor_msgs/NavSatFix",)


def wait_for_event(ev: threading.Event, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if ev.is_set():
            return True
        time.sleep(0.01)
    return ev.is_set()


class ClockMonitor:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.started = False
        self.first_clock_sec: Optional[float] = None
        self.last_clock_sec: Optional[float] = None
        self.last_clock_wall: Optional[float] = None

    def cb(self, msg: Clock) -> None:
        now_wall = time.monotonic()
        t = msg.clock.to_sec()
        with self._lock:
            if not self.started:
                self.started = True
                self.first_clock_sec = t
            self.last_clock_sec = t
            self.last_clock_wall = now_wall

    def has_stalled(self, stall_wall_sec: float) -> bool:
        with self._lock:
            if not self.started or self.last_clock_wall is None:
                return False
            return (time.monotonic() - self.last_clock_wall) > stall_wall_sec

    def get_first(self) -> Optional[float]:
        with self._lock:
            return self.first_clock_sec

    def get_last(self) -> Optional[float]:
        with self._lock:
            return self.last_clock_sec


class TFStampMonitor:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.first_ev = threading.Event()
        self.first_tf_stamp_sec: Optional[float] = None
        self.last_tf_stamp_sec: Optional[float] = None
        self.last_tf_wall: Optional[float] = None
        self.first_parent: Optional[str] = None
        self.first_child: Optional[str] = None

    def cb(self, msg: TFMessage) -> None:
        if not msg.transforms:
            return

        stamp_sec = msg.transforms[0].header.stamp.to_sec()
        now_wall = time.monotonic()

        with self._lock:
            if self.first_tf_stamp_sec is None:
                self.first_tf_stamp_sec = stamp_sec
                self.first_parent = msg.transforms[0].header.frame_id
                self.first_child = msg.transforms[0].child_frame_id
                self.first_ev.set()

            self.last_tf_stamp_sec = stamp_sec
            self.last_tf_wall = now_wall

    def get_first(self) -> Optional[float]:
        with self._lock:
            return self.first_tf_stamp_sec

    def get_last(self) -> Optional[float]:
        with self._lock:
            return self.last_tf_stamp_sec

    def has_stalled(self, stall_wall_sec: float) -> bool:
        with self._lock:
            if self.last_tf_wall is None:
                return False
            return (time.monotonic() - self.last_tf_wall) > stall_wall_sec

    def get_first_frames(self) -> Tuple[Optional[str], Optional[str]]:
        with self._lock:
            return self.first_parent, self.first_child


def wait_until_time_reached(
    t_query: float,
    max_wait_wall_sec: float,
    *,
    mode: str,
    clock_mon: Optional[ClockMonitor],
    tf_mon: TFStampMonitor,
    stall_wall_sec: float,
) -> str:
    deadline = time.monotonic() + max_wait_wall_sec
    tol = 1e-6

    while not rospy.is_shutdown():
        if mode == "CLOCK":
            if clock_mon is not None and clock_mon.has_stalled(stall_wall_sec):
                return "BAG_STOPPED"
            now_val = rospy.Time.now().to_sec()
        else:
            if tf_mon.has_stalled(stall_wall_sec):
                return "BAG_STOPPED"
            last_tf = tf_mon.get_last()
            now_val = last_tf if last_tf is not None else float("-inf")

        if now_val + tol >= t_query:
            return "REACHED"

        if time.monotonic() >= deadline:
            return "TIMEOUT"

        time.sleep(0.005)

    return "ROS_SHUTDOWN"


def current_topic_map() -> Dict[str, str]:
    try:
        topics = rospy.get_published_topics()
    except Exception:
        return {}
    return {name: type_name for name, type_name in topics}


def topic_score(name: str, topic_type: str, kind: str) -> int:
    lname = name.lower()
    score = 0
    if kind == "image":
        if topic_type == "sensor_msgs/CompressedImage":
            score += 100
        elif topic_type == "sensor_msgs/Image":
            score += 50
        if "compressed" in lname:
            score += 40
        if "compress" in lname:
            score += 35
        if "image" in lname:
            score += 20
        if "camera" in lname:
            score += 10
        if "raw" in lname:
            score -= 5
    else:
        if topic_type == "sensor_msgs/NavSatFix":
            score += 100
        if lname.endswith("/fix") or "/fix/" in lname:
            score += 50
        if "gps" in lname:
            score += 30
        if "gnss" in lname:
            score += 25
        if "navsat" in lname:
            score += 20
        if "nmea" in lname:
            score += 10
    return score


def best_topic(
    topic_map: Dict[str, str],
    *,
    allowed_types: Sequence[str],
    kind: str,
    override_name: Optional[str],
) -> Optional[Tuple[str, str]]:
    if override_name:
        topic_type = topic_map.get(override_name)
        if topic_type in allowed_types:
            return override_name, topic_type
        return None

    candidates: List[Tuple[int, str, str]] = []
    for name, topic_type in topic_map.items():
        if topic_type not in allowed_types:
            continue
        candidates.append((topic_score(name, topic_type, kind), name, topic_type))

    if not candidates:
        return None

    candidates.sort(key=lambda item: (item[0], item[1]), reverse=True)
    _, name, topic_type = candidates[0]
    return name, topic_type


def wait_for_topics(
    *,
    image_topic: Optional[str],
    gps_topic: Optional[str],
    timeout_sec: float,
) -> Tuple[Optional[Tuple[str, str]], Optional[Tuple[str, str]]]:
    image_override = image_topic if image_topic and image_topic.lower() != "auto" else None
    gps_override = gps_topic if gps_topic and gps_topic.lower() != "auto" else None

    deadline = time.monotonic() + timeout_sec
    last_summary: Optional[str] = None

    while not rospy.is_shutdown() and time.monotonic() < deadline:
        topic_map = current_topic_map()
        image_info = best_topic(
            topic_map,
            allowed_types=IMAGE_TOPIC_TYPES,
            kind="image",
            override_name=image_override,
        )
        gps_info = best_topic(
            topic_map,
            allowed_types=GPS_TOPIC_TYPES,
            kind="gps",
            override_name=gps_override,
        )

        summary = f"topics image={image_info} gps={gps_info}"
        if summary != last_summary:
            rospy.loginfo(summary)
            last_summary = summary

        if image_info is not None and gps_info is not None:
            return image_info, gps_info

        time.sleep(0.1)

    topic_map = current_topic_map()
    return (
        best_topic(topic_map, allowed_types=IMAGE_TOPIC_TYPES, kind="image", override_name=image_override),
        best_topic(topic_map, allowed_types=GPS_TOPIC_TYPES, kind="gps", override_name=gps_override),
    )


@dataclass
class SampleEvent:
    kind: str
    t_query_sec: float
    gps: Optional[Dict[str, float]] = None
    image_msg: Optional[object] = None


class ImageExportWriter:
    def __init__(self, output_dir: Path, timestamps_csv: Path, jpeg_quality: int = 95, queue_size: int = 512) -> None:
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.timestamps_csv = timestamps_csv
        self.timestamps_csv.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.timestamps_csv.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["filename", "t_query_sec", "t_in_sec"])
        self._file.flush()
        self._lock = threading.Lock()
        self._bridge = CvBridge()
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), max(1, min(int(jpeg_quality), 100))]
        self._next_index = 1
        self._first_stamp: Optional[float] = None
        self.saved_count = 0
        self.dropped_count = 0
        self._queue: Queue[Optional[Tuple[object, float]]] = Queue(maxsize=max(int(queue_size), 1))
        self._worker = threading.Thread(target=self._worker_loop, name="image-export-writer", daemon=True)
        self._worker.start()

    def close(self) -> None:
        try:
            self._queue.put_nowait(None)
        except Full:
            while True:
                try:
                    self._queue.get_nowait()
                except Empty:
                    break
                else:
                    self._queue.task_done()
            self._queue.put_nowait(None)
        self._worker.join(timeout=10.0)
        with self._lock:
            try:
                self._file.close()
            except Exception:
                pass

    def _normalize_for_jpeg(self, image: np.ndarray) -> np.ndarray:
        array = np.asarray(image)
        if array.size == 0:
            raise ValueError("Image payload is empty.")

        if array.ndim == 3 and array.shape[2] == 4:
            array = cv2.cvtColor(array, cv2.COLOR_BGRA2BGR)
        elif array.ndim == 3 and array.shape[2] == 1:
            array = array[:, :, 0]

        if array.dtype != np.uint8:
            finite = np.isfinite(array)
            if not np.any(finite):
                array = np.zeros(array.shape, dtype=np.uint8)
            else:
                valid = array[finite]
                min_val = float(np.nanmin(valid))
                max_val = float(np.nanmax(valid))
                if max_val > min_val:
                    array = np.clip((array - min_val) * 255.0 / (max_val - min_val), 0, 255).astype(np.uint8)
                else:
                    array = np.clip(array, 0, 255).astype(np.uint8)
        return array

    def _jpeg_bytes_from_message(self, msg: object) -> bytes:
        if isinstance(msg, CompressedImage):
            fmt = (msg.format or "").lower()
            if ("jpeg" in fmt or "jpg" in fmt) and msg.data:
                return bytes(msg.data)
            decoded = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            if decoded is None:
                raise ValueError("Failed to decode compressed image.")
            image = decoded
        elif isinstance(msg, Image):
            try:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception:
                image = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        else:
            raise TypeError(f"Unsupported image message type: {type(msg).__name__}")

        prepared = self._normalize_for_jpeg(np.asarray(image))
        ok, encoded = cv2.imencode(".jpg", prepared, self._jpeg_params)
        if not ok:
            raise ValueError("Failed to encode image as JPEG.")
        return encoded.tobytes()

    def _save(self, msg: object, stamp_sec: float) -> str:
        jpg_bytes = self._jpeg_bytes_from_message(msg)

        with self._lock:
            if self._first_stamp is None:
                self._first_stamp = stamp_sec
            t_in_sec = stamp_sec - self._first_stamp
            filename = f"frame_{self._next_index:06d}.jpg"
            self._next_index += 1

        output_path = self.output_dir / filename
        output_path.write_bytes(jpg_bytes)

        with self._lock:
            self._writer.writerow([filename, stamp_sec, t_in_sec])
            self._file.flush()
            self.saved_count += 1

        return filename

    def _worker_loop(self) -> None:
        while True:
            item = self._queue.get()
            try:
                if item is None:
                    return
                msg, stamp_sec = item
                self._save(msg, stamp_sec)
            except Exception as exc:
                rospy.logwarn(f"Failed to save image frame: {type(exc).__name__}: {exc}")
            finally:
                self._queue.task_done()

    def enqueue_save(self, msg: object, stamp_sec: float) -> bool:
        try:
            self._queue.put_nowait((msg, stamp_sec))
            return True
        except Full:
            with self._lock:
                self.dropped_count += 1
            return False


class TimebaseState:
    def __init__(self) -> None:
        self.initialized = False
        self.mode = "TF"
        self.first_tf: Optional[float] = None
        self.first_clock: Optional[float] = None
        self.lookup_time_offset_sec: float = 0.0


def build_gps_snapshot(msg: NavSatFix) -> Dict[str, float]:
    cov = list(msg.position_covariance)
    cov_xx = cov[0] if len(cov) >= 1 else float("nan")
    cov_yy = cov[4] if len(cov) >= 5 else float("nan")
    cov_zz = cov[8] if len(cov) >= 9 else float("nan")
    return {
        "latitude_deg": float(msg.latitude),
        "longitude_deg": float(msg.longitude),
        "altitude_m": float(msg.altitude),
        "fix_status": int(msg.status.status),
        "service": int(msg.status.service),
        "cov_xx_m2": float(cov_xx),
        "cov_yy_m2": float(cov_yy),
        "cov_zz_m2": float(cov_zz),
        "covariance_type": int(msg.position_covariance_type),
    }


def maybe_event_stamp(msg: object) -> Optional[float]:
    header = getattr(msg, "header", None)
    if header is None:
        return None
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    sec = float(stamp.to_sec())
    if sec <= 0.0:
        return None
    return sec


def initialize_timebase(
    *,
    timebase: TimebaseState,
    use_sim_time: bool,
    clock_ev: threading.Event,
    clock_mon: Optional[ClockMonitor],
    tf_mon: TFStampMonitor,
    wait_clock_wall_sec: float,
    wait_tf_wall_sec: float,
) -> Tuple[bool, str]:
    if timebase.initialized:
        return True, "OK"

    if use_sim_time:
        rospy.loginfo("use_sim_time is true; waiting for /clock to start (best-effort)...")
        wait_for_event(clock_ev, wait_clock_wall_sec)

    rospy.loginfo("Waiting for first /tf message...")
    if not wait_for_event(tf_mon.first_ev, wait_tf_wall_sec):
        return False, "FIRST_TF_TIMEOUT"

    first_tf = tf_mon.get_first()
    if first_tf is None:
        return False, "FIRST_TF_MISSING"

    first_clock = clock_mon.get_first() if clock_mon is not None else None
    now_clock = rospy.Time.now().to_sec()
    mode = "CLOCK"
    if first_tf > 1e6:
        mode = "TF"
    elif first_clock is None and use_sim_time:
        mode = "TF"

    parent, child = tf_mon.get_first_frames()
    rospy.loginfo(f"First TF: stamp={first_tf:.9f}, parent={parent}, child={child}")
    rospy.loginfo(f"Clock: first={first_clock}, Time.now()={now_clock:.9f}, mode={mode}")

    timebase.initialized = True
    timebase.mode = mode
    timebase.first_tf = first_tf
    timebase.first_clock = first_clock
    timebase.lookup_time_offset_sec = derive_lookup_time_offset_sec(first_tf, first_clock)
    if timebase.lookup_time_offset_sec:
        rospy.loginfo(
            "Applying TF lookup timestamp offset of %.9f seconds (clock -> TF domain).",
            timebase.lookup_time_offset_sec,
        )
    return True, "OK"


def timebase_stalled(mode: str, tf_mon: TFStampMonitor, clock_mon: Optional[ClockMonitor], stall_wall_sec: float) -> bool:
    if mode == "CLOCK":
        return bool(clock_mon is not None and clock_mon.has_stalled(stall_wall_sec))
    return tf_mon.has_stalled(stall_wall_sec)


def map_query_time_to_tf_domain(t_query_sec: float, timebase: TimebaseState) -> float:
    return map_query_time_to_tf_domain_impl(t_query_sec, timebase.lookup_time_offset_sec)


def map_tf_time_to_query_domain(t_tf_sec: float, timebase: TimebaseState) -> float:
    return map_tf_time_to_query_domain_impl(t_tf_sec, timebase.lookup_time_offset_sec)


def write_camera_row(
    writer: csv.writer,
    t_in_sec: float,
    t_query_sec: float,
    t_lookup_sec: float,
    pose: Sequence[object],
    status: str,
) -> None:
    writer.writerow([t_in_sec, t_query_sec, t_lookup_sec, *pose, status])


def write_gps_row(
    writer: csv.writer,
    t_in_sec: float,
    t_query_sec: float,
    pose: Sequence[object],
    status: str,
    gps: Optional[Dict[str, float]],
) -> None:
    snapshot = gps or {}
    writer.writerow(
        [
            t_in_sec,
            t_query_sec,
            *pose,
            status,
            snapshot.get("latitude_deg", ""),
            snapshot.get("longitude_deg", ""),
            snapshot.get("altitude_m", ""),
            snapshot.get("fix_status", ""),
            snapshot.get("service", ""),
            snapshot.get("cov_xx_m2", ""),
            snapshot.get("cov_yy_m2", ""),
            snapshot.get("cov_zz_m2", ""),
            snapshot.get("covariance_type", ""),
        ]
    )


class DenseTrajectorySampler:
    """Samples /tf at a fixed interval over the bag duration and writes tf_dense_trajectory.csv."""

    def __init__(
        self,
        out_path: Path,
        tf_buffer: "tf2_ros.Buffer",
        tf_mon: TFStampMonitor,
        use_sim_time: bool,
        clock_ev: threading.Event,
        clock_mon: Optional[ClockMonitor],
        wait_clock_wall_sec: float,
        wait_tf_wall_sec: float,
        target: str,
        source: str,
        interval_sec: float,
    ) -> None:
        self._out_path = out_path
        self._tf_buffer = tf_buffer
        self._tf_mon = tf_mon
        self._use_sim_time = use_sim_time
        self._clock_ev = clock_ev
        self._clock_mon = clock_mon
        self._wait_clock_wall_sec = wait_clock_wall_sec
        self._wait_tf_wall_sec = wait_tf_wall_sec
        self._target = target
        self._source = source
        self._interval_sec = max(interval_sec, 1e-4)
        self._thread: Optional[threading.Thread] = None
        self._file: Optional[object] = None
        self._writer: Optional[csv.writer] = None
        self.sample_count = 0
        self.fail_count = 0
        self.first_exported_timestamp_sec: Optional[float] = None
        self.last_exported_timestamp_sec: Optional[float] = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="dense-traj-sampler", daemon=True)
        self._thread.start()

    def close(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        if self._file is not None:
            try:
                self._file.close()  # type: ignore[union-attr]
            except Exception:
                pass

    def _run(self) -> None:
        timebase = TimebaseState()
        ok, status = initialize_timebase(
            timebase=timebase,
            use_sim_time=self._use_sim_time,
            clock_ev=self._clock_ev,
            clock_mon=self._clock_mon,
            tf_mon=self._tf_mon,
            wait_clock_wall_sec=self._wait_clock_wall_sec,
            wait_tf_wall_sec=self._wait_tf_wall_sec,
        )
        if not ok:
            rospy.logwarn(
                "[dense_traj] Failed to initialize timebase (%s); dense trajectory will be empty.",
                status,
            )
            return

        t_start = timebase.first_tf
        if t_start is None:
            rospy.logwarn("[dense_traj] No first TF stamp; dense trajectory will be empty.")
            return

        rospy.loginfo(
            "[dense_traj] Exporting dense trajectory in query/header time. "
            "first_tf=%.9f first_clock=%s lookup_offset=%.9f",
            t_start,
            "None" if timebase.first_clock is None else f"{timebase.first_clock:.9f}",
            timebase.lookup_time_offset_sec,
        )

        self._out_path.parent.mkdir(parents=True, exist_ok=True)
        with self._out_path.open("w", newline="", encoding="utf-8") as f:
            self._file = f
            writer = csv.writer(f)
            writer.writerow(["timestamp_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "status"])
            f.flush()

            t_cur = t_start
            # Small lag behind the live bag clock so we never try to look up a future transform.
            lag_sec = max(self._interval_sec * 2, 0.05)

            while not rospy.is_shutdown():
                last_tf = self._tf_mon.get_last()
                if last_tf is None:
                    time.sleep(self._interval_sec)
                    continue

                # Don't query further ahead than (last_known_tf - lag).
                t_safe = last_tf - lag_sec
                if t_cur > t_safe:
                    if self._tf_mon.has_stalled(stall_wall_sec=3.0):
                        # Bag finished — drain remaining samples up to last known stamp.
                        t_safe = last_tf
                        if t_cur > t_safe:
                            break
                    else:
                        time.sleep(self._interval_sec * 0.5)
                        continue

                stamp = rospy.Time.from_sec(t_cur)
                t_export = map_tf_time_to_query_domain(t_cur, timebase)
                try:
                    if self._tf_buffer.can_transform(self._target, self._source, stamp, rospy.Duration(0.0)):
                        tfm = self._tf_buffer.lookup_transform(
                            self._target, self._source, stamp, rospy.Duration(0.0)
                        )
                        tr = tfm.transform.translation
                        qr = tfm.transform.rotation
                        writer.writerow([t_export, tr.x, tr.y, tr.z, qr.x, qr.y, qr.z, qr.w, "OK"])
                        self.sample_count += 1
                    else:
                        writer.writerow([t_export, "", "", "", "", "", "", "", "NO_TF"])
                        self.fail_count += 1
                except Exception as exc:
                    status = f"FAIL:{type(exc).__name__}"
                    writer.writerow([t_export, "", "", "", "", "", "", "", status])
                    self.fail_count += 1

                if self.first_exported_timestamp_sec is None:
                    self.first_exported_timestamp_sec = t_export
                self.last_exported_timestamp_sec = t_export

                if self.sample_count % 100 == 0:
                    f.flush()

                t_cur += self._interval_sec

                # Exit once we are past the last observed TF stamp (bag finished).
                if self._tf_mon.has_stalled(stall_wall_sec=3.0) and t_cur > (self._tf_mon.get_last() or 0.0):
                    break

            f.flush()

        rospy.loginfo(
            "[dense_traj] Wrote %d samples (%d failures) to %s; exported timestamps [%.9f, %.9f]",
            self.sample_count,
            self.fail_count,
            self._out_path,
            -1.0 if self.first_exported_timestamp_sec is None else self.first_exported_timestamp_sec,
            -1.0 if self.last_exported_timestamp_sec is None else self.last_exported_timestamp_sec,
        )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera-out-csv", required=True)
    ap.add_argument("--gps-out-csv", required=True)
    ap.add_argument("--image-topic", default="")
    ap.add_argument("--gps-topic", default="")
    ap.add_argument("--image-output-dir", default="")
    ap.add_argument("--image-timestamps-csv", default="")
    ap.add_argument("--jpeg-quality", type=int, default=95)
    ap.add_argument("--target", default="camera_init")
    ap.add_argument("--source", default="body")
    ap.add_argument("--cache-sec", type=float, default=120.0)
    ap.add_argument("--wait-clock-wall-sec", type=float, default=10.0)
    ap.add_argument("--wait-tf-wall-sec", type=float, default=10.0)
    ap.add_argument("--discovery-timeout-wall-sec", type=float, default=15.0)
    ap.add_argument("--max-wait-per-sample-wall-sec", type=float, default=10.0)
    ap.add_argument("--retry-attempts", type=int, default=40)
    ap.add_argument("--retry-sleep-wall-sec", type=float, default=0.02)
    ap.add_argument("--stall-wall-sec", type=float, default=2.0)
    ap.add_argument("--event-queue-size", type=int, default=2048)
    ap.add_argument("--camera-time-offset-sec", type=float, default=0.0)
    ap.add_argument("--dense-traj-out-csv", default="")
    ap.add_argument("--dense-traj-interval-sec", type=float, default=0.010)
    args = ap.parse_args()

    rospy.init_node("tf_sample_camera_gps", anonymous=True, disable_signals=True)

    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(args.cache_sec))
    _listener = tf2_ros.TransformListener(tf_buffer)

    tf_mon = TFStampMonitor()
    _tf_sub = rospy.Subscriber("/tf", TFMessage, tf_mon.cb, queue_size=200)

    use_sim_time = bool(rospy.get_param("/use_sim_time", False))
    clock_mon: Optional[ClockMonitor] = None
    clock_ev = threading.Event()
    if use_sim_time:
        clock_mon = ClockMonitor()

        def _clock_cb(msg: Clock) -> None:
            clock_mon.cb(msg)
            clock_ev.set()

        _clock_sub = rospy.Subscriber("/clock", Clock, _clock_cb, queue_size=50)

    image_info, gps_info = wait_for_topics(
        image_topic=args.image_topic,
        gps_topic=args.gps_topic,
        timeout_sec=args.discovery_timeout_wall_sec,
    )

    if image_info is None:
        rospy.logwarn("No image topic detected; tf_camera_out.csv will contain only the header unless one appears later.")
    else:
        rospy.loginfo(f"Using image topic: {image_info[0]} ({image_info[1]})")

    if gps_info is None:
        rospy.logwarn("No NavSatFix topic detected; tf_gps_out.csv will contain only the header unless one appears later.")
    else:
        rospy.loginfo(f"Using GPS topic: {gps_info[0]} ({gps_info[1]})")

    if image_info is None and gps_info is None:
        print("No image or GPS topics could be detected.", file=sys.stderr)
        return 5

    dense_sampler: Optional[DenseTrajectorySampler] = None
    if args.dense_traj_out_csv:
        dense_sampler = DenseTrajectorySampler(
            out_path=Path(args.dense_traj_out_csv),
            tf_buffer=tf_buffer,
            tf_mon=tf_mon,
            use_sim_time=use_sim_time,
            clock_ev=clock_ev,
            clock_mon=clock_mon,
            wait_clock_wall_sec=args.wait_clock_wall_sec,
            wait_tf_wall_sec=args.wait_tf_wall_sec,
            target=args.target,
            source=args.source,
            interval_sec=args.dense_traj_interval_sec,
        )

    image_exporter: Optional[ImageExportWriter] = None
    if args.image_output_dir:
        if not args.image_timestamps_csv:
            print("--image-output-dir requires --image-timestamps-csv", file=sys.stderr)
            return 2
        image_exporter = ImageExportWriter(
            output_dir=Path(args.image_output_dir),
            timestamps_csv=Path(args.image_timestamps_csv),
            jpeg_quality=args.jpeg_quality,
            queue_size=max(args.event_queue_size, 1),
        )

    event_queue: Queue[SampleEvent] = Queue(maxsize=max(args.event_queue_size, 1))

    def enqueue_event(event: SampleEvent) -> bool:
        try:
            event_queue.put_nowait(event)
            return True
        except Full:
            rospy.logwarn_throttle(1.0, "Event queue is full; dropping event.")
            return False

    def image_cb(msg: object) -> None:
        stamp_sec = maybe_event_stamp(msg)
        if stamp_sec is None:
            rospy.logwarn_throttle(1.0, "Image message without a valid header stamp; skipping.")
            return
        enqueue_event(
            SampleEvent(
                kind="camera",
                t_query_sec=stamp_sec,
                image_msg=msg if image_exporter is not None else None,
            )
        )

    def gps_cb(msg: NavSatFix) -> None:
        stamp_sec = maybe_event_stamp(msg)
        if stamp_sec is None:
            rospy.logwarn_throttle(1.0, "GPS message without a valid header stamp; skipping.")
            return
        enqueue_event(SampleEvent(kind="gps", t_query_sec=stamp_sec, gps=build_gps_snapshot(msg)))

    image_sub = None
    gps_sub = None
    if image_info is not None:
        if image_info[1] == "sensor_msgs/CompressedImage":
            image_sub = rospy.Subscriber(image_info[0], CompressedImage, image_cb, queue_size=200)
        else:
            image_sub = rospy.Subscriber(image_info[0], Image, image_cb, queue_size=200)

    if gps_info is not None:
        gps_sub = rospy.Subscriber(gps_info[0], NavSatFix, gps_cb, queue_size=200)

    _ = image_sub, gps_sub

    counts = {"camera": 0, "gps": 0}

    if dense_sampler is not None:
        dense_sampler.start()

    try:
        with open(args.camera_out_csv, "w", newline="") as camera_f, open(args.gps_out_csv, "w", newline="") as gps_f:
            camera_writer = csv.writer(camera_f)
            gps_writer = csv.writer(gps_f)

            camera_writer.writerow(
                ["t_in_sec", "t_query_sec", "t_lookup_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
            )
            gps_writer.writerow(
                [
                    "t_in_sec",
                    "t_query_sec",
                    "x",
                    "y",
                    "z",
                    "qx",
                    "qy",
                    "qz",
                    "qw",
                    "status",
                    "latitude_deg",
                    "longitude_deg",
                    "altitude_m",
                    "fix_status",
                    "service",
                    "cov_xx_m2",
                    "cov_yy_m2",
                    "cov_zz_m2",
                    "covariance_type",
                ]
            )
            camera_f.flush()
            gps_f.flush()

            first_event_by_kind: Dict[str, float] = {}
            timebase = TimebaseState()

            while not rospy.is_shutdown():
                try:
                    event = event_queue.get(timeout=0.1)
                except Empty:
                    if timebase.initialized and event_queue.empty() and timebase_stalled(timebase.mode, tf_mon, clock_mon, args.stall_wall_sec):
                        rospy.loginfo("Timebase stalled and no pending events remain; assuming bag replay finished.")
                        break
                    if not timebase.initialized and use_sim_time and clock_mon is not None and clock_mon.has_stalled(args.stall_wall_sec):
                        rospy.loginfo("Clock stalled before any events were processed; assuming bag replay finished.")
                        break
                    continue

                first_event_by_kind.setdefault(event.kind, event.t_query_sec)
                t_in_sec = event.t_query_sec - first_event_by_kind[event.kind]

                ok, init_status = initialize_timebase(
                    timebase=timebase,
                    use_sim_time=use_sim_time,
                    clock_ev=clock_ev,
                    clock_mon=clock_mon,
                    tf_mon=tf_mon,
                    wait_clock_wall_sec=args.wait_clock_wall_sec,
                    wait_tf_wall_sec=args.wait_tf_wall_sec,
                )
                if not ok:
                    pose = ["", "", "", "", "", "", ""]
                    effective_query_time_sec = (
                        event.t_query_sec + args.camera_time_offset_sec
                        if event.kind == "camera"
                        else event.t_query_sec
                    )
                    if event.kind == "camera":
                        write_camera_row(
                            camera_writer,
                            t_in_sec,
                            event.t_query_sec,
                            effective_query_time_sec,
                            pose,
                            init_status,
                        )
                        camera_f.flush()
                        counts["camera"] += 1
                        if image_exporter is not None and event.image_msg is not None:
                            if not image_exporter.enqueue_save(event.image_msg, event.t_query_sec):
                                rospy.logwarn_throttle(1.0, "Image save queue is full; dropping image frame after TF sampling.")
                    else:
                        write_gps_row(gps_writer, t_in_sec, event.t_query_sec, pose, init_status, event.gps)
                        gps_f.flush()
                        counts["gps"] += 1
                    event_queue.task_done()
                    continue

                effective_query_time_sec = (
                    event.t_query_sec + args.camera_time_offset_sec
                    if event.kind == "camera"
                    else event.t_query_sec
                )

                wait_status = wait_until_time_reached(
                    t_query=effective_query_time_sec,
                    max_wait_wall_sec=args.max_wait_per_sample_wall_sec,
                    mode=timebase.mode,
                    clock_mon=clock_mon,
                    tf_mon=tf_mon,
                    stall_wall_sec=args.stall_wall_sec,
                )

                lookup_time_sec = map_query_time_to_tf_domain(effective_query_time_sec, timebase)
                stamp = rospy.Time.from_sec(lookup_time_sec)
                pose = ["", "", "", "", "", "", ""]
                status = wait_status

                if wait_status in ("REACHED", "BAG_STOPPED"):
                    attempts = 1 if wait_status == "BAG_STOPPED" else args.retry_attempts
                    status = "NO_TF_AVAILABLE"

                    for _ in range(attempts):
                        if rospy.is_shutdown():
                            status = "ROS_SHUTDOWN"
                            break

                        try:
                            if tf_buffer.can_transform(args.target, args.source, stamp, rospy.Duration(0.0)):
                                tfm = tf_buffer.lookup_transform(args.target, args.source, stamp, rospy.Duration(0.0))
                                tr = tfm.transform.translation
                                qr = tfm.transform.rotation
                                pose = [tr.x, tr.y, tr.z, qr.x, qr.y, qr.z, qr.w]
                                status = "OK"
                                break
                            status = "NO_TF_AVAILABLE"
                        except (LookupException, ConnectivityException) as exc:
                            status = f"LOOKUP_FAIL:{type(exc).__name__}"
                        except ExtrapolationException:
                            status = "EXTRAPOLATION"
                        except Exception as exc:
                            status = f"ERROR:{type(exc).__name__}"

                        if attempts > 1:
                            time.sleep(args.retry_sleep_wall_sec)

                    if wait_status == "BAG_STOPPED" and status != "OK":
                        status = f"BAG_STOPPED:{status}"
                else:
                    status = f"TIME_NOT_REACHED:{wait_status}"

                if event.kind == "camera":
                    write_camera_row(
                        camera_writer,
                        t_in_sec,
                        event.t_query_sec,
                        effective_query_time_sec,
                        pose,
                        status,
                    )
                    camera_f.flush()
                    counts["camera"] += 1
                    if image_exporter is not None and event.image_msg is not None:
                        if not image_exporter.enqueue_save(event.image_msg, event.t_query_sec):
                            rospy.logwarn_throttle(1.0, "Image save queue is full; dropping image frame after TF sampling.")
                else:
                    write_gps_row(gps_writer, t_in_sec, event.t_query_sec, pose, status, event.gps)
                    gps_f.flush()
                    counts["gps"] += 1

                event_queue.task_done()
    finally:
        if image_exporter is not None:
            image_exporter.close()
        if dense_sampler is not None:
            dense_sampler.close()

    message = (
        f"Wrote camera samples to {args.camera_out_csv} ({counts['camera']} rows) and "
        f"GPS samples to {args.gps_out_csv} ({counts['gps']} rows)."
    )
    if image_exporter is not None:
        message += f" Saved {image_exporter.saved_count} JPG frames to {args.image_output_dir}."
        if image_exporter.dropped_count:
            message += f" Dropped {image_exporter.dropped_count} image frames due to a full save queue."
    if dense_sampler is not None:
        message += (
            f" Dense trajectory: {dense_sampler.sample_count} samples"
            f" ({dense_sampler.fail_count} failures) to {args.dense_traj_out_csv}."
        )
    rospy.loginfo(message)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
