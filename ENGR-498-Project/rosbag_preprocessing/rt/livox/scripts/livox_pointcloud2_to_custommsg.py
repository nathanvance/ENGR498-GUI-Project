#!/usr/bin/env python3
from __future__ import annotations

import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from livox_ros_driver.msg import CustomMsg, CustomPoint
from sensor_msgs.msg import PointCloud2


class LivoxPointCloudRelay:
    def __init__(self) -> None:
        self.input_topic = rospy.get_param("~input_topic", "/livox/lidar_raw")
        self.output_topic = rospy.get_param("~output_topic", "/livox/lidar")
        self.default_span_ns = int(rospy.get_param("~default_span_ns", 100_000_000))
        self.max_span_ns = int(rospy.get_param("~max_span_ns", 200_000_000))
        self.last_stamp_ns: int | None = None

        self.publisher = rospy.Publisher(self.output_topic, CustomMsg, queue_size=4)
        self.subscriber = rospy.Subscriber(self.input_topic, PointCloud2, self._callback, queue_size=4)

        rospy.loginfo(
            "Relaying Livox-style PointCloud2 from %s to %s as livox_ros_driver/CustomMsg",
            self.input_topic,
            self.output_topic,
        )

    def _estimate_span_ns(self, stamp_ns: int) -> int:
        if self.last_stamp_ns is None:
            return self.default_span_ns
        delta = stamp_ns - self.last_stamp_ns
        if delta <= 0:
            return self.default_span_ns
        return min(delta, self.max_span_ns)

    def _callback(self, message: PointCloud2) -> None:
        field_names = [field.name for field in message.fields]
        required_fields = ("x", "y", "z", "intensity", "tag", "line")
        missing_fields = [name for name in required_fields if name not in field_names]
        if missing_fields:
            rospy.logerr_throttle(
                5.0,
                "Cannot convert %s because required fields are missing: %s",
                self.input_topic,
                ", ".join(missing_fields),
            )
            return

        points = list(pc2.read_points(message, field_names=required_fields, skip_nans=False))
        if not points:
            return

        stamp_ns = message.header.stamp.to_nsec()
        span_ns = self._estimate_span_ns(stamp_ns)
        converted = CustomMsg()
        converted.header = message.header
        converted.timebase = max(stamp_ns - span_ns, 0)
        converted.point_num = len(points)
        converted.lidar_id = 0
        converted.rsvd = [0, 0, 0]

        denom = max(len(points) - 1, 1)
        converted.points = []
        for index, point in enumerate(points):
            custom_point = CustomPoint()
            custom_point.offset_time = int(round(index * span_ns / denom))
            custom_point.x = float(point[0])
            custom_point.y = float(point[1])
            custom_point.z = float(point[2])

            intensity = 0.0 if point[3] is None else float(point[3])
            if math.isnan(intensity) or math.isinf(intensity):
                intensity = 0.0
            custom_point.reflectivity = max(0, min(255, int(round(intensity))))
            custom_point.tag = max(0, min(255, int(point[4])))
            custom_point.line = max(0, min(255, int(point[5])))
            converted.points.append(custom_point)

        self.publisher.publish(converted)
        self.last_stamp_ns = stamp_ns


def main() -> None:
    rospy.init_node("livox_pointcloud2_to_custommsg")
    LivoxPointCloudRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
