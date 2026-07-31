# Contributing

This repo starts as a skeleton on purpose. It defines the contract — topics, vocabulary, parameters, constraints — and you bring the implementation to it. That way research code becomes a DEXI package without the contract being reverse-engineered from it afterward.

## Porting your work in

**1. Fork and branch.** Branch off `main`, one feature per PR.

**2. Put the recognizer behind the existing seam.** `GestureRecognizer` in `src/hand_gesture_node.py` has three methods. Replace the bodies, keep the interface:

```python
def __init__(self, model_path, min_gesture_score, logger)
def classify(self, frame, timestamp_ms) -> str   # a value from VOCABULARY
def close(self)                                  # release model resources
```

`classify()` receives a decoded BGR frame and monotonic milliseconds. It returns a string from `VOCABULARY` — never a raw label from an upstream library. If your recognizer needs more than a few files, add a `dexi_hand_gesture/` Python module and import it; leave the node thin.

**3. Don't change what the node already gets right.** The QoS, frame-dropping, timestamp derivation, vote smoothing, per-frame publish, and `close()`-in-`finally` all exist because of specific bench failures documented in the README. If one of them is genuinely wrong, say so in the PR and we'll change it deliberately.

**4. Keep models out of git.** `*.task`, `*.tflite`, and `*.pkl` are gitignored. Document where the model lives and how to fetch it; point `model_path` at it.

**5. Pin what has to be pinned.** Anything without a rosdep key goes in `requirements.txt` with an exact version and a comment explaining the pin. `mediapipe==0.10.18` is the one that already bit us — it is the last `linux-aarch64` wheel.

**6. Leave training and experiments upstream.** This repo is the deployable node. Dataset builders, notebooks, and alternate pipelines stay in your research repo — link to it rather than porting it.

## Before you open the PR

- [ ] `colcon build --packages-select dexi_hand_gesture` clean
- [ ] Launches with no arguments: `ros2 launch dexi_hand_gesture hand_gesture_launch.py`
- [ ] `ros2 topic hz /hand_gesture_detections` shows the expected rate
- [ ] Only vocabulary values published — no raw library labels
- [ ] Ctrl-C exits promptly, and `systemctl restart` works (nothing hangs teardown)
- [ ] CPU measured on the target hardware and reported in the PR, not estimated
- [ ] README parameter table matches the actual parameters

State the hardware you tested on. "Works on my laptop" and "works on a 2GB CM5 under `dexi.service`" are different claims, and only the second one counts.

## Package conventions

Modeled on `dexi_color_detection` — match it when in doubt.

```
package.xml          ament_cmake + ament_cmake_python
CMakeLists.txt       installs src/ PROGRAMS, launch/, config/
config/              one params yaml, commented
launch/              *_launch.py, loads config by default
src/                 executable nodes
examples/python/     runnable subscribers
examples/node_red/   importable flows
```

Nodes report; they don't decide. This package publishes what it sees and never commands the drone. What `none` should mean to an offboard controller — hold, hold-with-timeout, or stop — is a flight-safety decision made downstream, by the PIC.
