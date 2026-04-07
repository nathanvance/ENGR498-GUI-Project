#!/usr/bin/env python3
import argparse
import csv
import sys
import time
import threading
from typing import Dict, List, Tuple, Optional

import rospy
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock


def _as_int(s: str) -> int:
    return int(str(s).strip())


def _as_float(s: str) -> float:
    return float(str(s).strip())


def csv_header_fields(csv_path: str) -> List[str]:
    with open(csv_path, "r", newline="") as f:
        r = csv.reader(f)
        header = next(r, None)
    return [h.strip() for h in header] if header else []


def load_timestamps(csv_path: str) -> List[Tuple[float, Dict[str, str]]]:
    """
    Returns list of (t_value, row_dict).

    Supported CSV formats:
      A) secs,nsecs   -> ABSOLUTE stamps (same units as TF header stamps)
      B) t            -> RELATIVE seconds from start
      C) stamp        -> RELATIVE seconds from start
    """
    out: List[Tuple[float, Dict[str, str]]] = []
    with open(csv_path, "r", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError("CSV has no header row.")

        fields = [h.strip() for h in reader.fieldnames]
        has_secs_nsecs = ("secs" in fields and "nsecs" in fields)
        has_t = ("t" in fields)
        has_stamp = ("stamp" in fields)

        if not (has_secs_nsecs or has_t or has_stamp):
            raise ValueError("CSV must include (secs,nsecs) or a single column named 't' or 'stamp'.")

        for row in reader:
            row_norm = {k.strip(): (v.strip() if isinstance(v, str) else v) for k, v in row.items()}
            if has_secs_nsecs:
                secs = _as_int(row_norm["secs"])
                nsecs = _as_int(row_norm["nsecs"])
                t = float(secs) + float(nsecs) * 1e-9
            elif has_t:
                t = _as_float(row_norm["t"])
            else:
                t = _as_float(row_norm["stamp"])

            out.append((t, row_norm))
    return out


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
            if not self.started:
                return False
            if self.last_clock_wall is None:
                return False
            return (time.monotonic() - self.last_clock_wall) > stall_wall_sec

    def get_first(self) -> Optional[float]:
        with self._lock:
            return self.first_clock_sec

    def get_last(self) -> Optional[float]:
        with self._lock:
            return self.last_clock_sec


class TFStampMonitor:
    """
    Tracks first and latest TF header stamps so we can drive timing off TF stamps
    even if /clock is missing or in a different timebase.
    """
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
        t = msg.transforms[0].header.stamp.to_sec()
        now_wall = time.monotonic()

        with self._lock:
            if self.first_tf_stamp_sec is None:
                self.first_tf_stamp_sec = t
                self.first_parent = msg.transforms[0].header.frame_id
                self.first_child = msg.transforms[0].child_frame_id
                self.first_ev.set()

            # track latest
            self.last_tf_stamp_sec = t
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
    """
    mode:
      - "CLOCK" : use rospy.Time.now() and /clock stall detection
      - "TF"    : use latest TF header stamp and /tf stall detection

    Returns: REACHED, TIMEOUT, BAG_STOPPED, ROS_SHUTDOWN
    """
    deadline = time.monotonic() + max_wait_wall_sec
    tol = 1e-6

    while not rospy.is_shutdown():
        if mode == "CLOCK":
            if clock_mon is not None and clock_mon.has_stalled(stall_wall_sec):
                return "BAG_STOPPED"
            now_val = rospy.Time.now().to_sec()
        else:
            # TF-driven
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


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--in_csv", required=True)
    ap.add_argument("--out_csv", required=True)
    ap.add_argument("--target", default="camera_init")
    ap.add_argument("--source", default="body")
    ap.add_argument("--cache_sec", type=float, default=120.0)
    ap.add_argument("--wait_clock_wall_sec", type=float, default=10.0)
    ap.add_argument("--wait_tf_wall_sec", type=float, default=10.0)
    ap.add_argument("--max_wait_per_sample_wall_sec", type=float, default=10.0)
    ap.add_argument("--retry_attempts", type=int, default=40)
    ap.add_argument("--retry_sleep_wall_sec", type=float, default=0.02)
    ap.add_argument("--stall_wall_sec", type=float, default=2.0)
    ap.add_argument("--sort_times", action="store_true")
    args = ap.parse_args()

    rospy.init_node("tf_sample_csv", anonymous=True, disable_signals=True)

    fields = csv_header_fields(args.in_csv)
    using_relative = ("t" in fields) or ("stamp" in fields)
    using_absolute = ("secs" in fields and "nsecs" in fields)
    if not (using_relative or using_absolute):
        print("CSV header must include either (secs,nsecs) or (t) or (stamp).", file=sys.stderr)
        return 2

    try:
        requests = load_timestamps(args.in_csv)
    except Exception as e:
        print(f"Failed to read timestamps: {e}", file=sys.stderr)
        return 2
    if not requests:
        print("No timestamps found.", file=sys.stderr)
        return 2
    if args.sort_times:
        requests.sort(key=lambda x: x[0])

    # TF2 buffer/listener
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(args.cache_sec))
    _listener = tf2_ros.TransformListener(tf_buffer)

    # Monitors
    tf_mon = TFStampMonitor()
    _tf_sub = rospy.Subscriber("/tf", TFMessage, tf_mon.cb, queue_size=200)

    use_sim_time = bool(rospy.get_param("/use_sim_time", False))
    clock_mon: Optional[ClockMonitor] = None
    clock_ev = threading.Event()

    if use_sim_time:
        clock_mon = ClockMonitor()

        def _clock_cb(msg: Clock):
            clock_mon.cb(msg)
            clock_ev.set()

        _clock_sub = rospy.Subscriber("/clock", Clock, _clock_cb, queue_size=50)

        rospy.loginfo("use_sim_time is true; waiting for /clock to start (best-effort)...")
        # NOTE: we do NOT fail hard if /clock doesn't show up; TF-driven mode can still work.
        wait_for_event(clock_ev, args.wait_clock_wall_sec)

    rospy.loginfo("Waiting for first /tf message...")
    if not wait_for_event(tf_mon.first_ev, args.wait_tf_wall_sec):
        print("Timed out waiting for first /tf. Ensure TF is being published and bag is playing.", file=sys.stderr)
        return 4

    first_tf = tf_mon.get_first()
    if first_tf is None:
        print("Internal error: first TF stamp missing.", file=sys.stderr)
        return 4

    first_clock = clock_mon.get_first() if clock_mon is not None else None
    now_clock = rospy.Time.now().to_sec()

    # Decide which timebase to use for waiting + t0.
    # If TF stamp looks epoch-like (>> 1e6) and /clock is small (hundreds/seconds), TF is the correct timebase.
    mode = "CLOCK"
    if first_tf > 1e6:
        # TF is epoch-like; safest is TF-driven
        mode = "TF"
    elif first_clock is None and use_sim_time:
        # sim time but no clock -> also TF-driven
        mode = "TF"

    parent, child = tf_mon.get_first_frames()
    rospy.loginfo(f"First TF: stamp={first_tf:.9f}, parent={parent}, child={child}")
    rospy.loginfo(f"Clock: first={first_clock}, Time.now()={now_clock:.9f}, mode={mode}")

    # Choose t0 for relative CSV
    if using_relative:
        if mode == "TF":
            t0_sec = float(first_tf)
            rospy.loginfo(f"Relative timestamps: using t0 from TF stamp = {t0_sec:.9f}")
        else:
            # CLOCK mode
            if first_clock is None:
                t0_sec = float(rospy.Time.now().to_sec())
                rospy.logwarn(f"Relative timestamps: /clock missing; using t0=Time.now() = {t0_sec:.9f}")
            else:
                t0_sec = float(first_clock)
                rospy.loginfo(f"Relative timestamps: using t0 from first /clock = {t0_sec:.9f}")
    else:
        t0_sec = 0.0

    with open(args.out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_in_sec", "t_query_sec", "x", "y", "z", "qx", "qy", "qz", "qw", "status"])
        f.flush()

        for (t_in, _row) in requests:
            if rospy.is_shutdown():
                break

            t_query = (t0_sec + t_in) if using_relative else t_in

            # Wait until timebase reaches query stamp
            wait_status = wait_until_time_reached(
                t_query=t_query,
                max_wait_wall_sec=args.max_wait_per_sample_wall_sec,
                mode=mode,
                clock_mon=clock_mon,
                tf_mon=tf_mon,
                stall_wall_sec=args.stall_wall_sec,
            )

            if wait_status == "BAG_STOPPED":
                w.writerow([t_in, t_query, "", "", "", "", "", "", "", "BAG_STOPPED"])
                f.flush()
                rospy.logwarn("Detected timebase stall - assuming bag stopped. Exiting.")
                break

            if wait_status != "REACHED":
                w.writerow([t_in, t_query, "", "", "", "", "", "", "", f"TIME_NOT_REACHED:{wait_status}"])
                f.flush()
                if wait_status == "ROS_SHUTDOWN":
                    break
                continue

            stamp = rospy.Time.from_sec(t_query)

            # Non-blocking TF2 lookups; wall-clock retries
            last_status = "NO_TF_AVAILABLE"
            got = False
            for _ in range(args.retry_attempts):
                if rospy.is_shutdown():
                    last_status = "ROS_SHUTDOWN"
                    break

                # if bag stops mid-retry, exit
                if (mode == "TF" and tf_mon.has_stalled(args.stall_wall_sec)) or \
                   (mode == "CLOCK" and clock_mon is not None and clock_mon.has_stalled(args.stall_wall_sec)):
                    last_status = "BAG_STOPPED"
                    break

                try:
                    if tf_buffer.can_transform(args.target, args.source, stamp, rospy.Duration(0.0)):
                        tfm = tf_buffer.lookup_transform(args.target, args.source, stamp, rospy.Duration(0.0))
                        tr = tfm.transform.translation
                        qr = tfm.transform.rotation
                        w.writerow([t_in, t_query, tr.x, tr.y, tr.z, qr.x, qr.y, qr.z, qr.w, "OK"])
                        f.flush()
                        got = True
                        last_status = "OK"
                        break
                    else:
                        last_status = "NO_TF_AVAILABLE"
                except (LookupException, ConnectivityException) as e:
                    last_status = f"LOOKUP_FAIL:{type(e).__name__}"
                except ExtrapolationException:
                    last_status = "EXTRAPOLATION"
                except Exception as e:
                    last_status = f"ERROR:{type(e).__name__}"

                time.sleep(args.retry_sleep_wall_sec)

            if not got:
                w.writerow([t_in, t_query, "", "", "", "", "", "", "", last_status])
                f.flush()

            if last_status == "BAG_STOPPED":
                rospy.logwarn("Bag stopped during TF retries - exiting.")
                break

    rospy.loginfo(f"Wrote samples to: {args.out_csv}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
