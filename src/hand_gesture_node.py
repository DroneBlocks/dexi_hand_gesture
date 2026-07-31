#!/usr/bin/env python3
"""Publishes recognized hand gestures on /hand_gesture_detections.

This is a working skeleton. It subscribes, decodes frames, smooths, and
publishes on the real topic contract — but always classifies as NONE.
Drop a recognizer into GestureRecognizer.classify() to make it real.
See CONTRIBUTING.md.
"""

from collections import Counter, deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

# The published vocabulary. Never publish a recognizer's raw label — map it here.
#
# NONE and UNKNOWN are distinct and must stay that way. MediaPipe emits the
# category "None" for *hand visible, no gesture matched*, while the research
# sandbox used "none" for *no hand at all*. One capital letter apart, opposite
# meanings. A consumer testing == "none" would silently ignore every
# hand-present-but-unrecognized frame.
NONE = 'none'          # no hand in frame
UNKNOWN = 'unknown'    # hand present, no gesture matched or below min score

GESTURES = (
    'open_palm',
    'closed_fist',
    'pointing_up',
    'thumb_up',
    'thumb_down',
    'victory',
    'i_love_you',
)

VOCABULARY = (NONE, UNKNOWN) + GESTURES


class GestureRecognizer:
    """Seam for the actual recognizer. Replace the body, keep the interface.

    classify() must return a string from VOCABULARY and must not block longer
    than a frame interval. close() must release any model resources — a
    MediaPipe LIVE_STREAM task left open hangs interpreter teardown and makes
    the process ignore SIGTERM, which systemd reads as unrestartable.
    """

    def __init__(self, model_path, min_gesture_score, logger):
        self._logger = logger
        self._logger.warn(
            'No recognizer wired up — publishing "%s" for every frame. '
            'Implement GestureRecognizer.classify().' % NONE
        )

    def classify(self, frame, timestamp_ms):
        return NONE

    def close(self):
        pass


class HandGestureNode(Node):

    def __init__(self):
        super().__init__('hand_gesture')

        self.declare_parameter('input_topic', '/cam0/image_raw/compressed_2hz')
        self.declare_parameter('output_topic', '/hand_gesture_detections')
        self.declare_parameter('model_path', '')
        self.declare_parameter('min_gesture_score', 0.5)
        self.declare_parameter('vote_window', 3)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        vote_window = max(1, self.get_parameter('vote_window').value)

        self._votes = deque(maxlen=vote_window)
        self._busy = False
        self._last_stamp_ms = -1
        self._last_published = None

        self._recognizer = GestureRecognizer(
            self.get_parameter('model_path').value,
            self.get_parameter('min_gesture_score').value,
            self.get_logger(),
        )

        self._publisher = self.create_publisher(String, output_topic, 10)

        # camera_ros publishes best-effort. A default (reliable) subscription
        # connects, reports healthy, and receives nothing.
        self._subscription = self.create_subscription(
            CompressedImage, input_topic, self._on_image, qos_profile_sensor_data
        )

        self.get_logger().info('%s -> %s' % (input_topic, output_topic))

    def _on_image(self, msg):
        # Drop frames rather than queue them. Recognition runs slower than the
        # camera; an unbounded queue is what drives a CM5 into swap.
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

            gesture = self._recognizer.classify(frame, self._timestamp_ms(msg))
            if gesture not in VOCABULARY:
                self.get_logger().warn('Off-vocabulary label %r' % gesture)
                gesture = UNKNOWN

            self._publish(gesture)
        except Exception as exc:  # a bad frame must not kill the node
            self.get_logger().error('Recognition failed: %s' % exc)
        finally:
            self._busy = False

    def _timestamp_ms(self, msg):
        """Monotonic milliseconds derived from the image header.

        Recognizers that hold temporal state require strictly increasing
        timestamps. Tying them to the frame rather than the wall clock keeps
        them honest, with a guard for cameras that publish a zero stamp.
        """
        stamp = msg.header.stamp
        ms = stamp.sec * 1000 + stamp.nanosec // 1_000_000
        if ms <= self._last_stamp_ms:
            ms = self._last_stamp_ms + 1
        self._last_stamp_ms = ms
        return ms

    def _publish(self, gesture):
        """Majority vote over the window, then publish.

        One message per processed frame, never on a timer. Message *arrival* is
        the freshness signal: if this node or the camera dies the topic goes
        quiet, and consumers can fail safe on a ~1s timeout. A heartbeat would
        destroy that property.
        """
        self._votes.append(gesture)
        winner = Counter(self._votes).most_common(1)[0][0]

        if winner != self._last_published:
            self.get_logger().info('gesture: %s' % winner)
            self._last_published = winner

        self._publisher.publish(String(data=winner))

    def close(self):
        self._recognizer.close()


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
