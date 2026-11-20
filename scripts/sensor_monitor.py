#!/usr/bin/env python3
"""Print one line every 10 s: each sensor's message rate, and PROBLEM if one is missing or too slow.

  15:30:20  OK       OAK-L 30.0  OAK-R 30.0  OAK-IMU 401  ZED-L 30.0  ZED-R 30.0  ZED-IMU 405  Lidar 9.9  THETA 5.0
  15:30:30  PROBLEM  THETA: no data | OAK-L 30.0  OAK-R 30.0  ...

Usage: python3 sensor_monitor.py
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, Imu, LaserScan

PERIOD = 10.0
# name, topic, type, slowest acceptable rate (Hz). The ZED is watched through camera_info, sent with every image.
SENSORS = [
    ('OAK-L', '/oak/left/image_rect', Image, 27),
    ('OAK-R', '/oak/right/image_rect', Image, 27),
    ('OAK-IMU', '/oak/imu/data', Imu, 360),
    ('ZED-L', '/zed2i/zed_node/left/camera_info', CameraInfo, 27),
    ('ZED-R', '/zed2i/zed_node/right/camera_info', CameraInfo, 27),
    ('ZED-IMU', '/zed2i/zed_node/imu/data_raw', Imu, 360),
    ('Lidar', '/scan', LaserScan, 9),
    ('THETA', '/theta/image_raw/compressed', CompressedImage, 4.5),
]


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')
        self.counts = {name: 0 for name, *_ in SENSORS}
        for name, topic, msg_type, _ in SENSORS:
            self.create_subscription(msg_type, topic, lambda _m, n=name: self.count(n), qos_profile_sensor_data,
                                     raw=True)
        self.t0 = time.monotonic()
        self.create_timer(PERIOD, self.report)
        print('sensor_monitor: one line every %.0f s' % PERIOD, flush=True)

    def count(self, name):
        self.counts[name] += 1

    def report(self):
        now = time.monotonic()
        dt, self.t0 = now - self.t0, now
        rates, problems = [], []
        for name, _, _, min_hz in SENSORS:
            hz = self.counts[name] / dt
            self.counts[name] = 0
            rates.append('%s %s' % (name, '%.0f' % hz if hz >= 100 else '%.1f' % hz))
            if hz == 0:
                problems.append('%s: no data' % name)
            elif hz < min_hz:
                problems.append('%s: slow' % name)
        stamp = time.strftime('%H:%M:%S')
        if problems:
            print('%s  PROBLEM  %s | %s' % (stamp, ', '.join(problems), '  '.join(rates)), flush=True)
        else:
            print('%s  OK       %s' % (stamp, '  '.join(rates)), flush=True)


def main():
    rclpy.init()
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
