# Contributing

This repo starts as a skeleton on purpose. It defines the contract (topics, vocabulary, parameters) and you bring the implementation to it, so research code becomes a DEXI package without anyone reverse-engineering the contract from it later.

## Porting your work in

**1. Fork and branch.** Branch off `main`, one feature per PR.

**2. Put the recognizer behind the existing seam.** `GestureRecognizer` in `src/hand_gesture_node.py` has three methods. Replace the bodies, keep the interface:

```python
def __init__(self, model_path, min_gesture_score, logger)
def classify(self, frame, timestamp_ms) -> str   # a value from VOCABULARY
def close(self)                                  # release model resources
```

`classify()` gets a decoded BGR frame and monotonic milliseconds, and returns a string from `VOCABULARY` rather than a raw label from an upstream library. If your recognizer needs more than a few files, add a `dexi_hand_gesture/` Python module and import it so the node stays thin.

**3. Check with us before changing the plumbing.** The QoS, frame-dropping, timestamp derivation, vote smoothing, per-frame publish, and `close()`-in-`finally` are each there because of a specific bench failure written up in the README. If one of them is wrong, say so in the PR and we'll change it on purpose.

**4. Keep models out of git.** `*.task`, `*.tflite`, and `*.pkl` are gitignored. Document where the model lives and how to fetch it, then point `model_path` at it.

**5. Pin what has to be pinned.** Anything without a rosdep key goes in `requirements.txt` with an exact version and a comment explaining why. `mediapipe==0.10.18` is the one that already bit us, since it's the last `linux-aarch64` wheel.

**6. Leave training and experiments upstream.** This repo is the deployable node. Dataset builders, notebooks, and alternate pipelines can stay in your research repo with a link from here.

## Before you open the PR

- [ ] `colcon build --packages-select dexi_hand_gesture` clean
- [ ] Launches with no arguments: `ros2 launch dexi_hand_gesture hand_gesture_launch.py`
- [ ] `ros2 topic hz /hand_gesture_detections` shows the expected rate
- [ ] Only vocabulary values published, no raw library labels
- [ ] Ctrl-C exits promptly and `systemctl restart` works
- [ ] CPU measured on the target hardware and reported in the PR
- [ ] README parameter table matches the actual parameters

Say which hardware you tested on. A laptop and a 2GB CM5 running the full `dexi.service` stack are very different results.

## Package conventions

Modeled on `dexi_color_detection`, which is the one to copy when in doubt.

```
package.xml          ament_cmake + ament_cmake_python
CMakeLists.txt       installs src/ PROGRAMS, launch/, config/
config/              one params yaml, commented
launch/              *_launch.py, loads config by default
src/                 executable nodes
examples/python/     runnable subscribers
examples/node_red/   importable flows
```

This package publishes what it sees and doesn't command the drone. What `none` should mean to an offboard controller (hold, hold with timeout, or stop) gets decided downstream by the PIC.
