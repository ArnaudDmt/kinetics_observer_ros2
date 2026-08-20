# kinetics_observer_ros2

ROS 2 wrapper around `stateObservation::KineticsObserver`. Package has no
dependency on `mc_state_observation`; estimator configuration and all runtime
inputs arrive through ROS messages.

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

Generate self-contained input bag with tools in `test_state_obs_ros2`, then:

```bash
ros2 launch kinetics_observer_ros2 kinetics_replay.launch.py \
  input_bag:=/tmp/kinetics_from_bin \
  output_bag:=/tmp/kinetics_estimated_state
```

Output path must not already exist.
