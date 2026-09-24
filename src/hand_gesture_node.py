#!/usr/bin/env python3
from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CompressedImage

from dexi_interfaces.msg import HandGestureDetection

from classifier_module import GestureClassifier

class HandGestureNode(Node):

    def __init__(self):
        super().__init__('hand_gesture')

        self.declare_parameter('input_topic', '/cam0/image_raw/compressed_2hz')
        self.declare_parameter('output_topic', '/hand_gesture_detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('min_gesture_score', 0.25)
        self.declare_parameter('vote_window', 3)
        self.declare_parameter('proc_width', 320)

        self.declare_parameter('idle_timeout_sec', 1.0)
        self.declare_parameter('idle_publish_rate', 1.0)

        # Any gap between consecutive _on_image calls larger than this gets
        # logged as a warning, well before it's large enough to trip the
        # idle timeout below. Lets us see approaching gaps, not just the
        # moment they cross the idle_timeout_sec threshold.
        self.declare_parameter('frame_gap_warn_sec', 0.75)
        self._frame_gap_warn_sec = self.get_parameter('frame_gap_warn_sec').value

        self._idle_timeout_sec = self.get_parameter('idle_timeout_sec').value
        idle_publish_rate = self.get_parameter('idle_publish_rate').value

        self._last_frame_time = self.get_clock().now()
        self._idle_timer = self.create_timer(1.0 / idle_publish_rate, self._on_idle_timer)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_path = self.get_parameter('model_path').value
        min_gesture_score = self.get_parameter('min_gesture_score').value
        vote_window = max(1, self.get_parameter('vote_window').value)
        proc_width = self.get_parameter('proc_width').value

        self._votes = deque(maxlen=vote_window)
        self._last_stamp_ms = -1
        self._last_published = None

        self._recognizer = GestureClassifier(
            model_path=model_path if model_path else None,
            min_gesture_score=min_gesture_score,
            proc_width=proc_width,
        )

        self._publisher = self.create_publisher(HandGestureDetection, output_topic, 10)

        image_qos = QoSProfile(depth=5)
        image_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        image_qos.history = HistoryPolicy.KEEP_LAST

        self._subscription = self.create_subscription(
            CompressedImage, input_topic, self._on_image, image_qos
        )

        self.get_logger().info('%s -> %s' % (input_topic, output_topic))

    def _on_image(self, msg):
        now = self.get_clock().now()
        gap = (now - self._last_frame_time).nanoseconds / 1e9
        self._last_frame_time = now

        if gap > self._frame_gap_warn_sec:
            self.get_logger().warn(
                'Gap of %.2fs since previous _on_image call (input_topic may have stalled)' % gap
            )
        else:
            self.get_logger().debug('_on_image called, %.2fs since previous frame' % gap)

        if self._recognizer.is_busy():
            self.get_logger().debug('Recognizer still busy, dropping this frame')
            return

        try:
            frame = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                self.get_logger().warn('Failed to decode frame')
                return

            result = self._recognizer.process_on_frame(frame, self._timestamp_ms(msg))
        except Exception as exc:  # a bad frame must not kill the node
            self.get_logger().error('Recognition failed: %s' % exc)
            return

        self._publish(result)

    def _timestamp_ms(self, msg):
        stamp = msg.header.stamp
        ms = stamp.sec * 1000 + stamp.nanosec // 1_000_000
        if ms <= self._last_stamp_ms:
            ms = self._last_stamp_ms + 1
        self._last_stamp_ms = ms
        return ms

    def _publish(self, result):
        self._votes.append(result['gesture_label'])
        winner = Counter(self._votes).most_common(1)[0][0]

        if winner != self._last_published:
            self.get_logger().info('gesture: %s' % winner)
            self._last_published = winner
            self._votes.clear()

        self.get_logger().debug(
            f'publishing: {winner}, score: {result["gesture_score"]:.2f}, two_hand: {result["gesture_two_hand"]}',
            throttle_duration_sec=1.0,
        )

        try:
            msg = HandGestureDetection()
            msg.gesture_name = str(winner)
            msg.gesture_score = float(result['gesture_score'])
            msg.two_hand = bool(result['gesture_two_hand'])
            msg.bbox = [float(v) for v in result['bbox']]
            self._publisher.publish(msg)
        except Exception as exc:
            import traceback
            self.get_logger().error('Publish failed: %s' % exc)
            self.get_logger().error(traceback.format_exc())

    def _on_idle_timer(self):
        elapsed = (self.get_clock().now() - self._last_frame_time).nanoseconds / 1e9
        self.get_logger().debug('Idle timer tick, %.2fs since last frame' % elapsed)

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
        msg.bbox = [0.0, 0.0, 0.0, 0.0]
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