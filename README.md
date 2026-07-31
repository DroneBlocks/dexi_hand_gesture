# dexi_hand_gesture

Hand gesture recognition ROS2 package for DEXI. Subscribes to compressed camera images and publishes recognized gestures that can be consumed by Python scripts, Node-RED flows, or any ROS2 subscriber.

> **Status: skeleton.** The node builds, runs, and publishes on the real topic contract, but classifies every frame as `none`. The recognizer is not wired up yet — see [CONTRIBUTING.md](CONTRIBUTING.md).

## Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/cam0/image_raw/compressed_2hz` | `sensor_msgs/CompressedImage` | Subscribe | Throttled camera input, best-effort QoS |
| `/hand_gesture_detections` | `std_msgs/String` | Publish | Recognized gesture, one message per processed frame |

## Vocabulary

| Value | Meaning |
|-------|---------|
| `none` | No hand in frame |
| `unknown` | Hand present, but no gesture matched or below `min_gesture_score` |
| `open_palm` `closed_fist` `pointing_up` `thumb_up` `thumb_down` `victory` `i_love_you` | Recognized |

`none` and `unknown` are deliberately distinct and must stay that way. MediaPipe emits the category `"None"` for *hand visible, no gesture matched*; the research sandbox used `"none"` for *no hand at all*. One capital letter apart, opposite meanings, both appear live. A Node-RED flow testing `payload.data == "none"` would silently ignore every hand-present-but-unrecognized frame. **Never publish a recognizer's raw label** — map it into the vocabulary above.

## Quick Start

```bash
colcon build --packages-select dexi_hand_gesture
source install/setup.bash
ros2 launch dexi_hand_gesture hand_gesture_launch.py
```

```bash
ros2 topic echo /hand_gesture_detections
ros2 topic hz   /hand_gesture_detections     # expect ~2.0
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/cam0/image_raw/compressed_2hz` | Camera source |
| `output_topic` | `/hand_gesture_detections` | Gesture output |
| `model_path` | `/home/dexi/models/gesture_recognizer.task` | Recognizer model on the drone |
| `min_gesture_score` | `0.5` | Below this a hand reports `unknown` |
| `vote_window` | `3` | Majority vote over last N frames before publishing |

## Node-RED

With rosbridge running (`ros2 launch rosbridge_server rosbridge_websocket_launch.xml`):

1. Add a **ros2-subscriber** node, set topic to `/hand_gesture_detections`, type `std_msgs/msg/String`
2. Read the gesture from `msg.payload.data`
3. Switch on it and trigger any action

See `examples/node_red/hand_gesture_flow.json` for an importable flow. Node-RED runs containerized, so it reaches rosbridge at `ws://host.docker.internal:9090`, not `localhost`.

## Python Examples

```bash
# Print gestures, and warn when the topic goes quiet
python3 examples/python/hand_gesture_subscriber.py
```

## Constraints

These are requirements, not suggestions. Each was expensive to find on the bench, and the skeleton already satisfies them — keep it that way.

**Subscribe with best-effort QoS.** `camera_ros` publishes sensor-data QoS. A default (reliable) subscription connects, reports healthy, and receives nothing.

**Use the 2 Hz throttled topic.** Measured on a 2GB CM5 Lite running the full `dexi.service` vision stack:

| Source topic | CPU | Notes |
|--------------|-----|-------|
| `/cam0/image_raw/compressed` (30 Hz) | ~86% of one core | Overkill for a hand signal |
| `/cam0/image_raw/compressed_2hz` | **~10–13% of one core** | 48–52°C, no throttle |

The classifier is nearly free; the hand landmarker is the entire bill. The throttled topic, not a cheaper model, is what makes this affordable.

**Subscribe — don't open the camera.** `camera_node` holds `/dev/video1` and `/dev/media0` exclusively while `dexi.service` runs, so `picamera2` cannot open it at all.

**Drop frames, don't queue them.** Recognition runs slower than the camera. An unbounded queue is what drives a CM5 into swap.

**Close the recognizer on shutdown.** A MediaPipe `LIVE_STREAM` task left open hangs interpreter teardown — a test process ignored `SIGTERM` for seven minutes. systemd reads that as a node it cannot restart.

**Publish per frame, never on a timer.** Message *arrival* is the freshness signal. If the node or camera dies the topic goes quiet, and consumers fail safe on a ~1s timeout. A heartbeat would destroy that property.

**One recognition graph, not two.** A `GestureRecognizerResult` already carries `hand_landmarks` and `handedness`, so a separate `HandLandmarker` detects the same hand twice. When both share one in-flight flag, the first callback to return re-arms dispatch while the other is still working. On a CM5 that meant ~200% of a core, 512MB RSS, and 30fps decaying to 12 before wedging in swap. One graph: 86%, 237MB, flat 30fps.

## Dependencies

**MediaPipe's last `linux-aarch64` wheel is `0.10.18`.** Every later release, including 0.10.35, ships x86_64/macOS/Windows only. On the CM5 (Bookworm, Python 3.11) pin:

```bash
pip install "mediapipe==0.10.18"
```

`mediapipe` has no rosdep key, so `package.xml` does not declare it. Note also that `mediapipe` no longer depends on `protobuf` — a stray `from google.protobuf...` import fails with `ModuleNotFoundError: No module named 'google'` on any clean install.

How mediapipe gets onto the drone is **unresolved**: pip into system Python during provisioning (PEP 668 forces `--break-system-packages`), a venv sourced by the systemd unit, or a container. Each interacts differently with the `dexi_ws` colcon build and with `rclpy` imported from `ros2_jazzy`. Settle it before this lands in `dexi_bringup`.

## Hardware note

`cam0` on DEXI-3 is the **downward** AprilTag camera. Nobody flies by reaching under the drone, so gesture control implies a forward-facing camera — a hardware conversation upstream of this package. On the bench, pointing at the ceiling reads as `none`, which is correct behavior, not a bug.

## Related

- [`dexi_color_detection`](https://github.com/DroneBlocks/dexi_color_detection) — closest sibling; same camera-in, detections-out shape
- [`dexi_apriltag`](https://github.com/DroneBlocks/dexi_apriltag) — where the 2 Hz throttle lesson came from
- [`bentheperson1/gesture_research_dexi`](https://github.com/bentheperson1/gesture_research_dexi) — upstream research sandbox

## License

MIT
