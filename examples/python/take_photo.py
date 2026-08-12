#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from dexi_interfaces.msg import HandGestureDetection
from dexi_interfaces.srv import LEDRingColor

class TakePhotoClient(Node):
    def __init__(self):
        super().__init__('hand_gesture_subscriber')

        # Image callback gets its own group so it can run on a different
        # executor thread than the gesture/timer callbacks below, instead
        # of competing with them one-at-a-time on a single thread.
        self.image_callback_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            HandGestureDetection, '/hand_gesture_detections', self._on_gesture, 104
        )

        self.cam_subscription = self.create_subscription(
            CompressedImage, '/cam0/image_raw/compressed', self._on_image,
            qos_profile_sensor_data, callback_group=self.image_callback_group
        )
        self.camera_frame = None

        self.led_client = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')

        self.gesture_confirmation_count = 0
        self.gesture_confirmation_threshold = 15

        self.blink_timer = None
        self.blink_toggles_left = 0
        self.blink_state_active = False

        self.can_take_photo = True

        self.cooldown_timer = None
        self.cooldown_time = 5.0

        self.flash_timer = None

        self.min_confidence_threshold = 0.7

        while not self.led_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LED service not available, waiting again...')

        self.set_led_color("black")

    def set_led_color(self, color):
        request = LEDRingColor.Request()
        request.color = color

        future = self.led_client.call_async(request)
        future.add_done_callback(lambda f: self._on_led_response(f, color))

    def _on_led_response(self, future, color):
        result = future.result()
        if result is not None:
            self.get_logger().debug(f'LED set to {color}: {result.message}')
        else:
            self.get_logger().error(f'LED service call failed for color {color}')

    def _on_image(self, msg):
        self._last_frame_time = self.get_clock().now()

        try:
            incoming_frame = cv2.imdecode(
                np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR
            )
            if incoming_frame is None:
                self.get_logger().warn('Failed to decode frame')
                return

            self.camera_frame = incoming_frame
        except Exception as exc:
            self.get_logger().error('Camera error: %s' % exc)

    def _on_gesture(self, msg):
        if msg.gesture_name == "zoom" and msg.gesture_score >= self.min_confidence_threshold and self.can_take_photo:
            self.gesture_confirmation_count += 1

            if self.gesture_confirmation_count >= self.gesture_confirmation_threshold and self.blink_timer is None:
                self.start_countdown(3)
                self.get_logger().info('Gesture confirmed, starting countdown to take photo.')
        else:
            self.gesture_confirmation_count = 0

    def start_countdown(self, seconds):
        self.blink_toggles_left = seconds * 2
        self.blink_state_active = True
        self.set_led_color("white")
        self.blink_toggles_left -= 1

        self.blink_timer = self.create_timer(0.5, self._blink_step)

    def _blink_step(self):
        self.blink_state_active = not self.blink_state_active
        self.set_led_color("white" if self.blink_state_active else "black")
        self.blink_toggles_left -= 1

        if self.blink_toggles_left <= 0:
            self.blink_timer.cancel()
            self.blink_timer = None
            self.get_logger().info('Countdown finished, taking photo.')
            self.can_take_photo = False

            self.cooldown_timer = self.create_timer(self.cooldown_time, self._cooldown_step)

            self.set_led_color("blue")
            self.flash_timer = self.create_timer(0.25, self._clear_flash)

            self.get_logger().info('Saving photo from /cam0 topic...')

            photo_name = f'dexi_photo_{self.get_clock().now().to_msg().sec}.jpg'
            cv2.imwrite(photo_name, self.camera_frame)

    def _clear_flash(self):
        self.set_led_color("black")
        self.flash_timer.cancel()
        self.flash_timer = None

    def _cooldown_step(self):
        self.cooldown_timer.cancel()
        self.cooldown_timer = None
        self.set_led_color("black")
        self.can_take_photo = True
        self.get_logger().info('Cooldown finished, ready to take another photo.')


def main(args=None):
    rclpy.init(args=args)
    node = TakePhotoClient()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()