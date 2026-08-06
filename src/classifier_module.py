import os
import time
import math
from collections import deque, Counter

import cv2
import numpy as np
import joblib
import mediapipe as mp

class GestureClassifier:
	def __init__(self, model_path = None, min_gesture_score = None):
		if model_path:
			self.landmarker_model = model_path
			data_dir = os.path.dirname(os.path.abspath(model_path))
		else:
			data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")
			self.landmarker_model = os.path.join(data_dir, "hand_landmarker.task")

		self.classifier_model = os.path.join(data_dir, "gesture_classifier.joblib")

		min_detection_confidence = 0.5
		min_tracking_confidence = 0.5

		self.smooth_window = 2
		self.confidence_floor = min_gesture_score if min_gesture_score is not None else 0.5
		self.multi_confidence_floor = (min_gesture_score + 0.1) if min_gesture_score is not None else 0.6

		self.box_pad = 0.02

		self.latest = {
			"hands": [],
			"gesture": "none",
			"score": 0.0,
			"two_hand": False
		}

		self.single_history = deque(maxlen = self.smooth_window)
		self.multi_history = deque(maxlen = self.smooth_window)

		print("loaded libraries")

		bundle = joblib.load(self.classifier_model)

		self.clf_single = bundle["single"]
		self.clf_multi = bundle["multi"]
		self.multi_hand_gestures = bundle["multi_hand_gestures"]
		self.no_multi_label = bundle["no_multi_label"]

		self.single_labels = list(self.clf_single.classes_) if self.clf_single is not None else []
		self.multi_labels = list(self.clf_multi.classes_) if self.clf_multi is not None else []

		print("loaded gesture classifier bundle")

		base_options = mp.tasks.BaseOptions(model_asset_path = self.landmarker_model)
		options = mp.tasks.vision.HandLandmarkerOptions(
			base_options = base_options,
			running_mode = mp.tasks.vision.RunningMode.LIVE_STREAM,
			num_hands = 2,
			min_hand_detection_confidence = min_detection_confidence,
			min_tracking_confidence = min_tracking_confidence,
			result_callback = self.on_result,
		)

		self.landmarker = mp.tasks.vision.HandLandmarker.create_from_options(options)

		print("initialized mediapipe hand landmarker")

	def normalize_landmarks(self, landmarks):
		wrist_x = landmarks[0].x
		wrist_y = landmarks[0].y
		
		pts = []
		for lm in landmarks:
			pts.append([lm.x - wrist_x, lm.y - wrist_y])
		
		scale = math.hypot(pts[9][0], pts[9][1])
		if scale < 1e-6:
			scale = 1e-6
		
		feat = []
		for p in pts:
			feat.append(p[0] / scale)
			feat.append(p[1] / scale)
		
		return feat

	def normalize_two_hands(self, hand_a, hand_b):
		if hand_a[0].x > hand_b[0].x:
			hand_a, hand_b = hand_b, hand_a
		
		mid_x = (hand_a[0].x + hand_b[0].x) / 2
		mid_y = (hand_a[0].y + hand_b[0].y) / 2
		
		scale = math.hypot(hand_a[0].x - hand_b[0].x, hand_a[0].y - hand_b[0].y)
		if scale < 1e-6:
			scale = 1e-6
		
		feat = []
		for lm in list(hand_a) + list(hand_b):
			feat.append((lm.x - mid_x) / scale)
			feat.append((lm.y - mid_y) / scale)
		
		return feat

	def classify(self, clf, class_labels, feat):
		probs = clf.predict_proba([feat])[0]
		best_index = int(np.argmax(probs))
		
		return class_labels[best_index], float(probs[best_index])

	def make_hand_info(self, hand, handedness_label):
		xs = [lm.x for lm in hand]
		ys = [lm.y for lm in hand]
		
		return {
			"gesture": "none",
			"score": 0.0,
			"handedness": handedness_label,
			"box_x1": min(xs) - self.box_pad,
			"box_y1": min(ys) - self.box_pad,
			"box_x2": max(xs) + self.box_pad,
			"box_y2": max(ys) + self.box_pad
		}

	def set_latest(self, hands_out, gesture, score, two_hand):
		self.latest["hands"] = hands_out
		self.latest["gesture"] = gesture
		self.latest["score"] = score
		self.latest["two_hand"] = two_hand

	def vote(self, history, gesture, score, floor):
		if score >= floor:
			history.append(gesture)
		
		if len(history) == 0:
			return "none"
		
		return Counter(history).most_common(1)[0][0]

	def classify_single_mode(self, result, hands_out):
		self.multi_history.clear()
		
		if self.clf_single is None:
			self.set_latest(hands_out, "none", 0.0, False)
			return
		
		hand = result.hand_landmarks[0]
		
		feat = self.normalize_landmarks(hand)
		gesture, score = self.classify(self.clf_single, self.single_labels, feat)
		
		voted = self.vote(self.single_history, gesture, score, self.confidence_floor)
		
		hands_out[0]["gesture"] = voted
		hands_out[0]["score"] = score
		
		self.set_latest(hands_out, voted, score, False)

	def classify_multi_mode(self, result, hands_out):
		self.single_history.clear()
		
		if self.clf_multi is None:
			self.set_latest(hands_out, "none", 0.0, True)
			return
		
		feat = self.normalize_two_hands(result.hand_landmarks[0], result.hand_landmarks[1])
		gesture, score = self.classify(self.clf_multi, self.multi_labels, feat)
		
		voted = self.vote(self.multi_history, gesture, score, self.multi_confidence_floor)
		
		if voted == self.no_multi_label:
			voted = "none"
		
		for hand in hands_out:
			hand["gesture"] = voted
			hand["score"] = score
		
		self.set_latest(hands_out, voted, score, True)

	def on_result(self, result, output_image, timestamp_ms):
		if not result.hand_landmarks:
			self.single_history.clear()
			self.multi_history.clear()
			self.set_latest([], "none", 0.0, False)
			
			return
		
		hands_out = []
		for hand, handedness in zip(result.hand_landmarks, result.handedness):
			hands_out.append(self.make_hand_info(hand, handedness[0].category_name))
		
		if len(result.hand_landmarks) >= 2:
			self.classify_multi_mode(result, hands_out)
		else:
			self.classify_single_mode(result, hands_out)

	def process_on_frame(self, frame, timestamp_ms):
		frame = cv2.flip(frame, 1)
		rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
		mp_image = mp.Image(image_format = mp.ImageFormat.SRGB, data = rgb)

		self.landmarker.detect_async(mp_image, timestamp_ms)

		label = "none"
	
		hands = self.latest["hands"]
		
		if self.latest["two_hand"] and len(hands) >= 2:
			label = self.latest["gesture"]
		elif len(hands) >= 1:
			label = hands[0]["gesture"]

		return label