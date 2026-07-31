# dexi_hand_gesture

Hand gesture recognition ROS2 package for DEXI. Subscribes to compressed camera images and publishes recognized gestures that can be consumed by Python scripts, Node-RED flows, or any ROS2 subscriber.

> **Status: skeleton.** The node builds, runs, and publishes on the real topic contract, but classifies every frame as `none`. The recognizer isn't wired up yet. See [CONTRIBUTING.md](CONTRIBUTING.md).

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

`none` and `unknown` are separate values on purpose. MediaPipe emits the category `"None"` when a hand is visible but no gesture matched, while the research sandbox used `"none"` for no hand at all. They're one capital letter apart, mean opposite things, and both show up live. A Node-RED flow testing `payload.data == "none"` will silently ignore every hand-present-but-unrecognized frame. Map your recognizer's labels into the vocabulary above rather than publishing them raw.

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

## Bench notes

The skeleton already handles all of this. It's written down so nobody has to find it twice.

**Best-effort QoS on the subscription.** `camera_ros` publishes sensor-data QoS. A default reliable subscription connects, reports healthy, and receives nothing.

**Use the 2 Hz throttled topic.** Measured on a 2GB CM5 Lite running the full `dexi.service` vision stack:

| Source topic | CPU | Notes |
|--------------|-----|-------|
| `/cam0/image_raw/compressed` (30 Hz) | ~86% of one core | Overkill for a hand signal |
| `/cam0/image_raw/compressed_2hz` | **~10-13% of one core** | 48-52°C, no throttle |

The classifier is nearly free. The hand landmarker is the entire bill, which is why the throttled topic buys more than a cheaper model would.

**Subscribe instead of opening the camera.** `camera_node` holds `/dev/video1` and `/dev/media0` exclusively while `dexi.service` runs, so `picamera2` can't open it at all.

**Drop frames rather than queueing them.** Recognition runs slower than the camera, and an unbounded queue is what drove a CM5 into swap.

**Close the recognizer on shutdown.** A MediaPipe `LIVE_STREAM` task left open hangs interpreter teardown. One test process ignored `SIGTERM` for seven minutes, which systemd sees as a node it can't restart.

**Publish per frame, not on a timer.** Message arrival is the freshness signal, so a dead node or camera makes the topic go quiet and consumers can fail safe on a ~1s timeout. A heartbeat would take that away.

**One recognition graph, not two.** `GestureRecognizerResult` already carries `hand_landmarks` and `handedness`, so a separate `HandLandmarker` detects the same hand twice. With both sharing one in-flight flag, the first callback to return re-arms dispatch while the other is still working. On a CM5 that meant ~200% of a core, 512MB RSS, and 30fps decaying to 12 before wedging in swap. One graph: 86%, 237MB, flat 30fps.

## Dependencies

MediaPipe's last `linux-aarch64` wheel is `0.10.18`. Everything later, including 0.10.35, ships x86_64/macOS/Windows only. On the CM5 (Bookworm, Python 3.11):

```bash
pip install "mediapipe==0.10.18"
```

`mediapipe` has no rosdep key, so `package.xml` doesn't declare it. It also no longer depends on `protobuf`, so a stray `from google.protobuf...` import fails with `ModuleNotFoundError: No module named 'google'` on a clean install.

How mediapipe gets onto the drone is still open: pip into system Python during provisioning (PEP 668 forces `--break-system-packages`), a venv sourced by the systemd unit, or a container. Each one interacts differently with the `dexi_ws` colcon build and with `rclpy` imported from `ros2_jazzy`. Worth settling before this lands in `dexi_bringup`.

## Hardware note

`cam0` on DEXI-3 is the downward AprilTag camera, so gesture control implies a forward-facing camera and a hardware conversation upstream of this package. On the bench the camera points at the ceiling, which reads as `none`. That's correct behavior, not a bug.

## Related

- [`dexi_color_detection`](https://github.com/DroneBlocks/dexi_color_detection) - closest sibling, same camera-in detections-out shape
- [`dexi_apriltag`](https://github.com/DroneBlocks/dexi_apriltag) - where the 2 Hz throttle lesson came from
- [`bentheperson1/gesture_research_dexi`](https://github.com/bentheperson1/gesture_research_dexi) - upstream research sandbox

## License

MIT
