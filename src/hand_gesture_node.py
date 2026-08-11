#!/usr/bin/env python3
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from dexi_interfaces.msg import HandGestureDetection

from classifier_module import GestureClassifier

class HandGestureNode(Node):

    def __init__(self):
        super().__init__('hand_gesture')

        self.declare_parameter('input_topic', '/cam0/image_raw/compressed')
        self.declare_parameter('output_topic', '/hand_gesture_detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('min_gesture_score', 0.5)
        self.declare_parameter('vote_window', 3)

        self.declare_parameter('idle_timeout_sec', 1.0)
        self.declare_parameter('idle_publish_rate', 1.0)

        self._idle_timeout_sec = self.get_parameter('idle_timeout_sec').value
        idle_publish_rate = self.get_parameter('idle_publish_rate').value

        self._last_frame_time = self.get_clock().now()
        self._idle_timer = self.create_timer(1.0 / idle_publish_rate, self._on_idle_timer)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_path = self.get_parameter('model_path').value
        min_gesture_score = self.get_parameter('min_gesture_score').value
        vote_window = max(1, self.get_parameter('vote_window').value)

        self._votes = deque(maxlen=vote_window)
        self._busy = False
        self._last_stamp_ms = -1
        self._last_published = None

        self._recognizer = GestureClassifier(
            model_path=model_path if model_path else None,
            min_gesture_score=min_gesture_score,
        )

        self._publisher = self.create_publisher(HandGestureDetection, output_topic, 10)

        # camera_ros publishes best-effort. A default (reliable) subscription
        # connects, reports healthy, and receives nothing.
        self._subscription = self.create_subscription(
            CompressedImage, input_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info('%s -> %s' % (input_topic, output_topic))

    def _on_image(self, msg):
        self._last_frame_time = self.get_clock().now()

        if self._busy:
            return
        
        self._busy = True
        try:
            frame = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                self.get_logger().warn('Failed to decode frame')
                return

            result = self._recognizer.process_on_frame(frame, self._timestamp_ms(msg))

            self._publish(result)
        except Exception as exc:  # a bad frame must not kill the node
            self.get_logger().error('Recognition failed: %s' % exc)
        finally:
            self._busy = False

    def _timestamp_ms(self, msg):
        """Monotonic milliseconds derived from the image header.

        Recognizers that hold temporal state require strictly increasing
        timestamps. Deriving them from the frame rather than the wall clock
        keeps them tied to the image, with a guard for cameras that publish a
        zero stamp.
        """
        stamp = msg.header.stamp
        ms = stamp.sec * 1000 + stamp.nanosec // 1_000_000
        if ms <= self._last_stamp_ms:
            ms = self._last_stamp_ms + 1
        self._last_stamp_ms = ms
        return ms

    def _publish(self, result):
        """Majority vote over the window, then publish.

        One message per processed frame, not on a timer. Message arrival is the
        freshness signal, so if this node or the camera dies the topic goes
        quiet and consumers can fail safe on a ~1s timeout.
        """
        self._votes.append(result['gesture_label'])
        winner = Counter(self._votes).most_common(1)[0][0]

        if winner != self._last_published:
            self.get_logger().info('gesture: %s' % winner)
            self._last_published = winner

        self.get_logger().info('publishing: %s' % winner)

        msg = HandGestureDetection()
        msg.gesture_name = winner
        msg.gesture_score = result['gesture_score']
        msg.two_hand = result['gesture_two_hand']
        self._publisher.publish(msg)

    def _on_idle_timer(self):
        elapsed = (self.get_clock().now() - self._last_frame_time).nanoseconds / 1e9
        if elapsed < self._idle_timeout_sec:
            return

        if self._last_published != 'no_gesture':
            self.get_logger().warn('no frames for %.1fs, publishing no_gesture' % elapsed)
            self._last_published = 'no_gesture'
            self._votes.clear()

        msg = HandGestureDetection()
        msg.gesture_name = 'no_gesture'
        msg.gesture_score = 0.0
        msg.two_hand = False
        self._publisher.publish(msg)

    def close(self):
        self.get_logger().info('Closing hand gesture node')
        # self._recognizer.close()


def main(args=None):
    rclpy.init(args=args)
    node = HandGestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()