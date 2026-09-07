# kinetics_observer_ros2

ROS 2 wrapper around `stateObservation::KineticsObserver`. Package has no
dependency on `mc_state_observation`; estimator configuration and all runtime
inputs arrive through ROS messages.

## Repository roles

- `state-observation`: estimator library used by this wrapper.
- `mc_state_observation`: mc_rtc observer/plugin that runs the estimator and
  records `.bin` logs.
- `kinetics_observer_ros2`: ROS 2 wrapper, messages, and node.
- `test_state_obs_ros2`: `.bin` conversion, synchronized replay, inspection,
  and comparison plots.

## Topics

- Subscribe: `~/configuration` (`KineticsConfiguration`, reliable)
- Subscribe: `~/input` (`KineticsInput`, reliable)
- Publish: `~/estimated_state` (`KineticsState`, reliable)

Configuration constructs and resets estimator. Inputs received before valid
configuration are rejected. Contact IDs and IMU IDs are zero-based and must be
smaller than configured maxima.

`KineticsKinematics.valid_fields` preserves unset fields used by
state-observation finite-difference logic. Matrices are flattened row-major.
ROS quaternions use `x, y, z, w`.

## Build

```bash
colcon build --packages-select kinetics_observer_ros2 test_state_obs_ros2
# zsh
source install/setup.zsh
# bash users should source install/setup.bash instead
```

## Replay

Generate a self-contained input bag with `test_state_obs_ros2`, then launch the
ROS 2 node. The input bag contains configuration plus timestamped estimator
inputs; the output bag contains `/kinetics_observer/estimated_state`.

```bash
ros2 launch test_state_obs_ros2 test_kinetics_replay.launch.py \
  input_bag:=/tmp/kinetics_from_bin \
  output_bag:=/tmp/kinetics_estimated_state
```

Replay is synchronized: the next input is sent after the previous estimate is
received. There is no `play_rate` argument. If the requested output directory
already exists, the launch automatically appends `_1`, `_2`, and so on.
