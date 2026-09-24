#!/usr/bin/env python3

import os
import cv2
import rclpy
import numpy as np

from enum import Enum, auto

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import CompressedImage
from dexi_interfaces.msg import HandGestureDetection
from dexi_interfaces.srv import LEDRingColor, LEDPixelColor


class PhotoState(Enum):
	"""
	IDLE       -> waiting for a qualifying gesture. Gesture input starts
	              confirmation.
	CONFIRMING -> gesture must be held continuously while the LED ring
	              fills up. Losing the gesture cancels back to IDLE.
	              Reaching the end of the ring moves to COUNTDOWN.
	COUNTDOWN  -> gesture input is completely ignored. The blink timer
	              runs on its own until it reaches zero, then a photo
	              is taken.
	CAPTURING  -> transient state while the frame is written to disk.
	COOLDOWN   -> gesture input is ignored until the cooldown timer
	              elapses, then back to IDLE.
	"""
	IDLE = auto()
	CONFIRMING = auto()
	COUNTDOWN = auto()
	CAPTURING = auto()
	COOLDOWN = auto()


class TakePhotoClient(Node):
	def __init__(self, num_leds=34):
		super().__init__('take_photo_client')

		self.get_logger().info('Initializing TakePhotoClient node')

		self.image_callback_group = MutuallyExclusiveCallbackGroup()

		self.create_subscription(
			HandGestureDetection, '/hand_gesture_detections', self._on_gesture, 104
		)

		self.cam_subscription = self.create_subscription(
			CompressedImage, '/cam0/image_raw/compressed_2hz', self._on_image,
			qos_profile_sensor_data, callback_group=self.image_callback_group
		)
		self.camera_frame = None

		self.led_ring_client = self.create_client(LEDRingColor, '/dexi/led_service/set_led_ring_color')
		self.led_pixel_client = self.create_client(LEDPixelColor, '/dexi/led_service/set_led_pixel_color')

		self.state = PhotoState.IDLE

		self.gesture_confirmation_duration = 2.0
		self.countdown_seconds = 3
		self.cooldown_time = 5.0

		self.min_confidence_threshold = 0.7

		self.NUM_LEDS = num_leds
		self.current_led_index = 0

		self._confirmation_write_pending = False
		self._confirmation_generation = 0

		self.blink_timer = None
		self.blink_toggles_left = 0
		self.blink_state_active = False

		self.cooldown_timer = None
		self.flash_timer = None

		self.declare_parameter('photo_save_dir', '~/dexi_photos')
		self.photo_save_dir = os.path.expanduser(self.get_parameter('photo_save_dir').get_parameter_value().string_value)
		os.makedirs(self.photo_save_dir, exist_ok=True)
		self.get_logger().info(f'Photos will be saved to: {self.photo_save_dir}')

		while not self.led_ring_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('LED service not available, waiting again...')

		while not self.led_pixel_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('LED pixel service not available, waiting again...')

		self.set_led_ring_color("black")

		self.increment_progress_timer_rate = self.gesture_confirmation_duration / self.NUM_LEDS
		self.increment_progress_timer = self.create_timer(
			self.increment_progress_timer_rate, self._increment_progress, autostart=False
		)

	# ------------------------------------------------------------------
	# Gesture handling (IDLE / CONFIRMING only)
	# ------------------------------------------------------------------

	def _is_qualifying_gesture(self, msg):
		return msg.gesture_name == "fist" and msg.gesture_score >= self.min_confidence_threshold

	def _on_gesture(self, msg):
		qualifies = self._is_qualifying_gesture(msg)

		if self.state == PhotoState.IDLE:
			if qualifies:
				self._start_gesture_confirmation()

		elif self.state == PhotoState.CONFIRMING:
			if not qualifies:
				self._cancel_gesture_confirmation()

	def _start_gesture_confirmation(self):
		self.get_logger().info('Zoom gesture detected, starting confirmation window.')

		self._confirmation_generation += 1
		self._confirmation_write_pending = False

		self.state = PhotoState.CONFIRMING
		self.current_led_index = 0
		self.set_led_ring_color("black")

		self.increment_progress_timer.reset()

	def _cancel_gesture_confirmation(self):
		self.get_logger().info('Gesture lost before confirmation completed, canceling.')

		self.increment_progress_timer.cancel()

		self._confirmation_generation += 1
		self._confirmation_write_pending = False

		self._clear_confirmation_leds(self.current_led_index)
		self.set_led_ring_color("red")

		self.create_timer(0.5, lambda: self.set_led_ring_color("black"))

		self.current_led_index = 0
		self.state = PhotoState.IDLE

	def _clear_confirmation_leds(self, up_to_index):
		for i in range(up_to_index + 1):
			self.set_led_pixel_color(i, 0, 0, 0)

	def _increment_progress(self):
		if self.state != PhotoState.CONFIRMING:
			self.increment_progress_timer.cancel()
			return

		if self._confirmation_write_pending:
			return

		self._confirmation_write_pending = True
		generation = self._confirmation_generation
		self.set_led_pixel_color(
			self.current_led_index, 0, 255, 0,
			on_response=lambda index, success: self._on_confirmation_pixel_written(index, success, generation)
		)

	def _on_confirmation_pixel_written(self, index, success, generation):
		if generation != self._confirmation_generation:
			return

		self._confirmation_write_pending = False

		if self.state != PhotoState.CONFIRMING:
			return

		self.current_led_index = index + 1

		if self.current_led_index >= self.NUM_LEDS:
			self._confirm_gesture()

	def _confirm_gesture(self):
		self.get_logger().info('Gesture confirmed, starting countdown to take photo.')

		self.increment_progress_timer.cancel()
		self.current_led_index = 0

		self.state = PhotoState.COUNTDOWN
		self._start_countdown(self.countdown_seconds)

	# ------------------------------------------------------------------
	# Countdown -> capture -> cooldown (gesture-proof)
	# ------------------------------------------------------------------

	def _start_countdown(self, seconds):
		self.get_logger().info(f'Starting countdown for {seconds} seconds')

		if self.blink_timer is not None:
			self.blink_timer.cancel()
			self.blink_timer = None

		self.blink_toggles_left = seconds * 2
		self.blink_state_active = True
		self.set_led_ring_color("white")
		self.blink_toggles_left -= 1

		self.blink_timer = self.create_timer(0.5, self._blink_step)

	def _blink_step(self):
		if self.state != PhotoState.COUNTDOWN:
			if self.blink_timer is not None:
				self.blink_timer.cancel()
				self.blink_timer = None
			return

		self.blink_state_active = not self.blink_state_active
		self.set_led_ring_color("white" if self.blink_state_active else "black")
		self.blink_toggles_left -= 1

		if self.blink_toggles_left <= 0:
			self.blink_timer.cancel()
			self.blink_timer = None
			self.get_logger().info('Countdown finished, taking photo.')
			self._take_photo()

	def _take_photo(self):
		self.state = PhotoState.CAPTURING

		self.set_led_ring_color("blue")

		self.get_logger().info('Saving photo from /cam0 topic...')

		if self.camera_frame is None:
			self.get_logger().warn('camera_frame is None, photo save will likely fail')

		photo_name = f'dexi_photo_{self.get_clock().now().to_msg().sec}.jpg'
		photo_path = os.path.join(self.photo_save_dir, photo_name)

		self.get_logger().info(f'Writing photo to {photo_path}')

		success = cv2.imwrite(photo_path, self.camera_frame)
		if success:
			self.get_logger().info(f'Photo saved to {photo_path}')
		else:
			self.get_logger().error(f'cv2.imwrite reported failure writing {photo_path}')

		if self.flash_timer is not None:
			self.flash_timer.cancel()
			self.flash_timer = None
		self.flash_timer = self.create_timer(0.25, self._clear_flash)

		self._start_cooldown()

	def _clear_flash(self):
		self.set_led_ring_color("black")

		self.flash_timer.cancel()
		self.flash_timer = None

	def _start_cooldown(self):
		if self.cooldown_timer is not None:
			self.cooldown_timer.cancel()
			self.cooldown_timer = None

		self.state = PhotoState.COOLDOWN
		self.cooldown_timer = self.create_timer(self.cooldown_time, self._cooldown_step)

	def _cooldown_step(self):
		self.cooldown_timer.cancel()
		self.cooldown_timer = None

		self.state = PhotoState.IDLE
		self.set_led_ring_color("black")
		self.get_logger().info('Cooldown finished, ready to take another photo.')

	# ------------------------------------------------------------------
	# LED helpers
	# ------------------------------------------------------------------

	def set_led_pixel_color(self, index, r, g, b, on_response=None):
		request = LEDPixelColor.Request()
		request.index = index
		request.r = r
		request.g = g
		request.b = b

		future = self.led_pixel_client.call_async(request)
		future.add_done_callback(lambda f: self._on_led_pixel_response(f, index, r, g, b, on_response))

	def _on_led_pixel_response(self, future, index, r, g, b, on_response=None):
		result = future.result()
		if result is not None:
			self.get_logger().debug(f'LED pixel set to index {index} with color ({r}, {g}, {b}): {result.message}')
		else:
			self.get_logger().error(f'LED pixel service call failed for index {index} with color ({r}, {g}, {b})')

		if on_response is not None:
			on_response(index, result is not None)

	def set_led_ring_color(self, color):
		request = LEDRingColor.Request()
		request.color = color

		future = self.led_ring_client.call_async(request)
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
			incoming_frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
			if incoming_frame is None:
				self.get_logger().warn('Failed to decode frame')
				return

			self.camera_frame = incoming_frame
		except Exception as exc:
			self.get_logger().error('Camera error: %s' % exc)


def main(args=None):
	rclpy.init(args=args)
	node = TakePhotoClient()

	executor = MultiThreadedExecutor()
	executor.add_node(node)

	node.get_logger().debug('Starting executor spin')

	try:
		executor.spin()
	except KeyboardInterrupt:
		node.get_logger().debug('KeyboardInterrupt received, shutting down')
	finally:
		node.get_logger().debug('Shutting down: destroying node')
		node.destroy_node()

		if rclpy.ok():
			rclpy.shutdown()


if __name__ == '__main__':
	main()