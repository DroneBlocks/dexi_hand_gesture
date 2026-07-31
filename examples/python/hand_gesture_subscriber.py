#!/usr/bin/env python3
"""Print hand gestures, and flag the topic going quiet.

Nothing is published on a timer, so silence means the node or the camera died.
Treat it as a fault rather than assuming the last gesture still holds.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TIMEOUT_SEC = 1.0


class GestureSubscriber(Node):

    def __init__(self):
        super().__init__('hand_gesture_subscriber')
        self._last_seen = self.get_clock().now()
        self._stale = False
        self.create_subscription(
            String, '/hand_gesture_detections', self._on_gesture, 10
        )
        self.create_timer(0.5, self._check_stale)

    def _on_gesture(self, msg):
        self._last_seen = self.get_clock().now()
        if self._stale:
            self.get_logger().info('Topic recovered')
            self._stale = False
        self.get_logger().info(msg.data)

    def _check_stale(self):
        age = (self.get_clock().now() - self._last_seen).nanoseconds / 1e9
        if age > TIMEOUT_SEC and not self._stale:
            self.get_logger().warn('No gesture for %.1fs, treating as fault' % age)
            self._stale = True


def main(args=None):
    rclpy.init(args=args)
    node = GestureSubscriber()
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
