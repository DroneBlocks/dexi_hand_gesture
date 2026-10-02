# Contributing

Thanks for helping improve `dexi_hand_gesture`. This repo is the **deployable ROS 2 node**: it runs MediaPipe hand landmark detection plus a trained classifier onboard a DEXI and publishes the result on `/hand_gesture_detections`. Contributions are welcome for the node, the examples, the docs, and new gesture workflows.

Please read the [README](README.md) first for how the node works and what its topics and parameters are.

## Where things belong

| I want to... | Do it in... |
|--------------|-------------|
| Add or retrain gestures (capture, annotate, train) | The [training repo](https://github.com/bentheperson1/dexi_hand_gesture_train). Bring the resulting `gesture_classifier.joblib` here |
| Change detection, classification, or smoothing behavior | `src/classifier_module.py` / `src/hand_gesture_node.py` (this repo) |
| Show how to use gestures | `examples/python/` or `examples/node_red/` (this repo) |
| Run experiments, notebooks, or alternate pipelines | Your own research repo, with a link from here |

This repo is deliberately not a place for dataset builders or training code. Keep it deployable.

## Making a change

1. **Fork and branch** off `main`. One feature or fix per PR.
2. **Keep the node thin.** `hand_gesture_node.py` handles ROS plumbing (subscriptions, publishing, timers, parameters). Detection and classification logic lives in `GestureClassifier` in `classifier_module.py`. If you replace or extend the recognizer, preserve its interface:

   ```python
   GestureClassifier(model_path, min_gesture_score, proc_width)
   process_on_frame(frame, timestamp_ms) -> dict   # gesture_label, gesture_score,
                                                   # gesture_two_hand, bbox
   is_busy() -> bool                               # True while a detection is in flight
   ```

   `process_on_frame()` receives a decoded BGR frame and a strictly increasing millisecond timestamp. If your recognizer needs more than a couple of files, add a Python module and import it rather than growing the node.
3. **Check with us before changing the plumbing.** Several behaviors exist because of specific failures on the target hardware, and each has a comment in the code explaining why. Open an issue or say so in the PR if you think one is wrong, so it gets changed on purpose:
   - Best-effort QoS on the camera subscription
   - Dropping frames while the landmarker is busy (`is_busy()`), rather than queueing them. Queued detections can exhaust CPU/RAM and lock up a 2 GB CM5
   - Downscaling to `proc_width` before detection
   - Monotonic timestamp derivation in `_timestamp_ms()`
   - Confidence floor plus vote smoothing before publishing
   - The idle timer that publishes `no_gesture` when frames stop
   - Closing the node cleanly in `finally`
4. **Don't hard-code gesture names in the node.** Labels come from the trained model, so the node should publish whatever the model predicts, plus `no_gesture`. Gesture-specific behavior (like "fist takes a photo") belongs in consumers and examples, not in the node.
5. **Keep the normalization in sync with training.** `normalize_landmarks()` and `normalize_two_hands()` in `classifier_module.py` must produce exactly the same features as the versions in the training repo. If you change one, change the other, or every existing model will silently get worse.
6. **Keep large and generated files out of git.** `gesture_classifier.joblib` and `landmarks_cache.npz` are not committed. `data/hand_landmarker.task` is the only model file tracked in this repo. Document where any additional model lives and how to fetch it, then point `model_path` at it.
7. **Pin what has to be pinned.** Anything without a rosdep key goes in `requirements.txt` with an exact version and a comment saying why. `mediapipe==0.10.18` is the one that has already caused trouble, since it's the last `linux-aarch64` wheel. scikit-learn is also sensitive: a `.joblib` only loads reliably with the scikit-learn version that trained it, so change it here and in the training repo together.

## Package conventions

Modeled on `dexi_color_detection`, which is the one to copy when in doubt.

```
package.xml          ament_cmake + ament_cmake_python
CMakeLists.txt       installs src/ PROGRAMS, launch/, config/
config/              one params yaml, commented
data/                hand_landmarker.task (tracked); trained .joblib (not tracked)
launch/              *_launch.py, loads config by default
src/                 executable nodes and the classifier module
examples/python/     runnable subscribers
examples/node_red/   importable flows
```

- New examples that should work with `ros2 run` must be added to the `install(PROGRAMS ...)` block in `CMakeLists.txt` and be executable.
- If you add or change a parameter, update `config/hand_gesture_params.yaml` and the README parameter table in the same PR.

This package publishes what it sees and doesn't command the drone. What `no_gesture` should mean to an offboard controller (hold, hold with a timeout, or stop) is decided downstream by the pilot in command.

## Environment setup

After cloning and before building:

    pip3 install --break-system-packages -r src/dexi_hand_gesture/requirements.txt

This overrides apt's system-installed `scipy`, which predates NumPy 2.x and breaks the `joblib.load()` -> sklearn -> scipy import chain with `AttributeError: _ARRAY_API not found`. Do this before `colcon build`.

Then:

    colcon build --symlink-install
    source install/setup.bash
    ros2 launch dexi_hand_gesture hand_gesture_launch.py

You will need a trained `gesture_classifier.joblib` in the data directory before the node will start. See the README for details.

## Before you open the PR

- [ ] `colcon build --packages-select dexi_hand_gesture` is clean
- [ ] Launches with no arguments: `ros2 launch dexi_hand_gesture hand_gesture_launch.py`
- [ ] `ros2 topic hz /hand_gesture_detections` shows the expected rate
- [ ] Both single-hand and two-hand gestures were tested, along with the no-hand case (`no_gesture`)
- [ ] Only labels from the model, plus `no_gesture`, are published
- [ ] Ctrl-C exits promptly and `systemctl restart` works
- [ ] CPU measured on the target hardware and reported in the PR
- [ ] Any changed example still runs, including via `ros2 run` if it is installed
- [ ] README parameter table and examples section match the code

Say which hardware you tested on. A laptop and a 2 GB CM5 running the full `dexi.service` stack are very different results.