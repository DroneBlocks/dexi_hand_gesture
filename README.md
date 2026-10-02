# dexi_hand_gesture

A ROS 2 node that recognizes hand gestures in real time from the DEXI camera feed and publishes the result as a ROS message. Any other node (flight logic, LED effects, Node-RED, etc.) can subscribe to it and react to gestures.

Detection uses [MediaPipe Hand Landmarker](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) to find 21 landmarks per hand, then a pre-trained scikit-learn Random Forest to classify the gesture from those landmarks. The classifier model is produced by the companion [model training repo](https://github.com/bentheperson1/dexi_hand_gesture_train) (see [Using your own gestures](#using-your-own-gestures)).

## How it works

```
 /cam0/image_raw/compressed_2hz
            |
            v
   +------------------+     +---------------------+     +-----------------------+
   | hand_gesture     | --> | MediaPipe Hand      | --> | Random Forest         |
   | node (decode,    |     | Landmarker          |     | (single-hand or       |
   | resize, flip)    |     | (async, 21 pts/hand)|     |  two-hand model)      |
   +------------------+     +---------------------+     +-----------------------+
                                                                    |
                                                                    v
                                              vote / confidence filter
                                                                    |
                                                                    v
                                         /hand_gesture_detections
                                         (dexi_interfaces/HandGestureDetection)
```

- **One hand visible** -> the single-hand model classifies it.
- **Two hands visible** -> the two-hand model is used. If it predicts `none`, the result is reported as `no_gesture`.
- Predictions below the confidence floor are discarded, and a short majority vote over recent frames smooths the output.
- Frames are dropped (not queued) while the landmarker is still busy, so the node can't fall behind and starve the board's CPU/RAM.
- If no frames arrive for `idle_timeout_sec`, the node publishes `no_gesture` on a timer so downstream nodes never get stuck on a stale gesture.

## Repository layout

```
dexi_hand_gesture/
├── config/
│   └── hand_gesture_params.yaml    # node parameters
├── data/
│   ├── hand_landmarker.task        # MediaPipe model (in repo)
│   └── gesture_classifier.joblib   # trained classifier (you provide, see below)
├── examples/
│   ├── node_red/                   # Node-RED example
│   └── python/
│       ├── gesture_yaw_track.py    # yaw tracking example
│       └── take_photo.py           # fist-to-take-a-photo example
├── launch/
│   └── hand_gesture_launch.py
├── resource/
├── src/
│   ├── classifier_module.py        # MediaPipe + classifier logic
│   └── hand_gesture_node.py        # ROS 2 node
├── CMakeLists.txt
├── package.xml
└── requirements.txt
```

## Setup

### Requirements

- A DEXI with a working camera publishing `sensor_msgs/CompressedImage`
- ROS 2 workspace on the drone (default location `~/dexi_ws`)
- The `dexi_interfaces` package (provides `HandGestureDetection`)
- Python dependencies from `requirements.txt` (MediaPipe, OpenCV, NumPy, scikit-learn, joblib)

### Install

```bash
cd ~/dexi_ws/src
git clone https://github.com/DroneBlocks/dexi_hand_gesture.git
cd dexi_hand_gesture
pip install -r requirements.txt

cd ~/dexi_ws
colcon build --packages-select dexi_hand_gesture
source install/setup.bash
```

### Add a trained model

The node needs `gesture_classifier.joblib` in the `data/` directory next to `hand_landmarker.task`:

```
~/dexi_ws/dexi_hand_gesture/data/gesture_classifier.joblib
```

Copy it over with `scp` or by dragging it in through VS Code.

> **Version match:** `.joblib` files are tied to the scikit-learn version that created them. Train with the same scikit-learn version that is installed on the drone, otherwise loading may fail or give wrong results.

## Running

```bash
ros2 launch dexi_hand_gesture hand_gesture_launch.py
```

Watch the output:

```bash
ros2 topic echo /hand_gesture_detections
```

The node logs each time the detected gesture changes (`gesture: fist`).

## Topics

| Direction | Topic (default) | Type | Description |
|-----------|-----------------|------|-------------|
| Subscribes | `/cam0/image_raw/compressed_2hz` | `sensor_msgs/CompressedImage` | Camera frames (best-effort QoS) |
| Publishes | `/hand_gesture_detections` | `dexi_interfaces/HandGestureDetection` | Current gesture |

### `HandGestureDetection` message

| Field | Type | Description |
|-------|------|-------------|
| `gesture_name` | string | Gesture label from your model, or `no_gesture` |
| `gesture_score` | float | Classifier probability for that label (0-1) |
| `two_hand` | bool | `true` if the two-hand model produced the result |
| `bbox` | float[4] | Bounding box `[x1, y1, x2, y2]` in pixels of the original frame. All zeros when no hand is found. Because frames are mirrored before detection, x values are in the mirrored image |

## Parameters

Set in `config/hand_gesture_params.yaml`, or override at launch.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `input_topic` | `/cam0/image_raw/compressed_2hz` | Camera topic to subscribe to |
| `output_topic` | `/hand_gesture_detections` | Topic to publish detections on |
| `model_path` | `''` | Path to `hand_landmarker.task`. The classifier `.joblib` is expected in the same directory. Empty looks for a `data/` directory next to the installed scripts, so set this to the repo's `data/` directory unless `data/` is installed with the package |
| `min_gesture_score` | `0.25` | Confidence floor for single-hand predictions. The two-hand floor is this value + 0.1 |
| `vote_window` | `3` | Number of recent frames in the majority vote. Higher is steadier but slower to react |
| `proc_width` | `320` | Frames wider than this are downscaled before landmark detection. Lower is faster, higher is more accurate at range |
| `idle_timeout_sec` | `1.0` | Time without frames before publishing `no_gesture` |
| `idle_publish_rate` | `1.0` | Rate (Hz) of the idle `no_gesture` messages |
| `frame_gap_warn_sec` | `0.75` | Logs a warning when the camera feed stalls longer than this |

### Tuning tips

- **Missing real gestures:** lower `min_gesture_score`.
- **False triggers:** raise `min_gesture_score` and/or `vote_window`. Consumers can also apply their own threshold on `gesture_score` (the photo example uses 0.7).
- **High CPU use or lag:** lower `proc_width`.
- The default camera topic runs at 2 Hz, so expect up to roughly half a second of latency per decision. A larger `vote_window` adds to that.

## Examples

All examples live in `examples/` and subscribe to `/hand_gesture_detections`.

### Running the Python examples with `ros2 run`

The Python examples are installed with the package, so once it's built and sourced you can run them with `ros2 run` from any directory. The gesture node must be running first (see [Running](#running)), so use two terminals (or run the launch file in the background).

```bash
# Terminal 1: gesture detection
source ~/dexi_ws/install/setup.bash
ros2 launch dexi_hand_gesture hand_gesture_launch.py

# Terminal 2: an example
source ~/dexi_ws/install/setup.bash
ros2 run dexi_hand_gesture take_photo.py
```

Pass parameters with `--ros-args`:

```bash
ros2 run dexi_hand_gesture take_photo.py --ros-args -p photo_save_dir:=~/my_photos
```

Notes:

- The package name is `dexi_hand_gesture` and the executable name is the script's filename, `.py` included. Press Tab after `ros2 run dexi_hand_gesture` to list what was installed.
- Only scripts listed in the `install(PROGRAMS ...)` block of `CMakeLists.txt` can be run with `ros2 run`. Currently that is `take_photo.py` (plus `hand_gesture_node.py`). To make another example runnable, such as `gesture_yaw_track.py`, add it to that block:

  ```cmake
  install(PROGRAMS
    src/hand_gesture_node.py
    src/classifier_module.py
    examples/python/take_photo.py
    examples/python/gesture_yaw_track.py
    DESTINATION lib/${PROJECT_NAME}
  )
  ```

  Then make sure the file is executable (`chmod +x`), rebuild with `colcon build --packages-select dexi_hand_gesture`, and re-source `install/setup.bash`.
- You can always run a script directly instead, e.g. `python3 examples/python/take_photo.py`, as long as the workspace is sourced.

### Take a photo with a fist (`examples/python/take_photo.py`)

Hold a **fist** (score >= 0.7) and the LED ring fills up over 2 seconds. Dropping the gesture early cancels and flashes the ring red. Once the ring is full, the ring blinks through a 3 second countdown, a photo is saved from the camera, and there is a 5 second cooldown before the next one. Gestures are ignored during the countdown and cooldown.

```bash
ros2 run dexi_hand_gesture take_photo.py --ros-args -p photo_save_dir:=~/dexi_photos
```

Requires the DEXI LED services (`/dexi/led_service/set_led_ring_color` and `/dexi/led_service/set_led_pixel_color`). Photos are saved to `~/dexi_photos` by default.

### Yaw tracking (`examples/python/gesture_yaw_track.py`)

Uses the detected hand position to drive yaw tracking. See the script header for usage.

### Node-RED (`examples/node_red/`)

Example Node-RED flow for reacting to gestures with a visual programming workflow.

## Using your own gestures

The node can run any gesture set you train. To make your own model:

1. Use the [dexi_hand_gesture_train](https://github.com/bentheperson1/dexi_hand_gesture_train) repo to capture images, extract landmarks, and train a classifier.
2. Copy the resulting `gesture_classifier.joblib` into this repo's `data/` directory on the drone (see [Add a trained model](#add-a-trained-model)).
3. Restart the node. New gesture names will appear in `gesture_name` automatically, so update any consumers (like the examples) to match your labels.

## Troubleshooting

| Symptom | Likely cause |
|---------|--------------|
| Node crashes at startup with a `FileNotFoundError` | `gesture_classifier.joblib` or `hand_landmarker.task` is missing from `data/` (or from the directory in `model_path`) |
| Errors loading the `.joblib` | scikit-learn version mismatch between training machine and drone |
| Constant `no_gesture` | Camera topic isn't publishing, hand is out of frame, or `min_gesture_score` is too high |
| `Gap of X s since previous _on_image call` warnings | The camera topic stalled. Check `ros2 topic hz` on the input topic |
| Gesture flickers | Raise `vote_window` or `min_gesture_score`; collect more varied training data |
| Very high CPU or the board becomes unresponsive | Lower `proc_width` and confirm no other heavy nodes are running |

## License

See [LICENSE](LICENSE). Contributions are welcome, see [CONTRIBUTING.md](CONTRIBUTING.md).