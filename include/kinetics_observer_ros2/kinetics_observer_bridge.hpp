#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include "kinetics_observer_ros2/msg/kinetics_configuration.hpp"
#include "kinetics_observer_ros2/msg/kinetics_input.hpp"
#include "kinetics_observer_ros2/msg/kinetics_kinematics.hpp"
#include "kinetics_observer_ros2/msg/kinetics_state.hpp"

namespace kinetics_observer_ros2
{

class KineticsObserverBridge
{
public:
  using Configuration = msg::KineticsConfiguration;
  using Input = msg::KineticsInput;
  using State = msg::KineticsState;

  void configure(const Configuration & configuration);
  State update(const Input & input);

  bool configured() const noexcept { return static_cast<bool>(observer_); }

private:
  struct ActiveContact
  {
    std::string name;
  };

  std::unique_ptr<stateObservation::KineticsObserver> observer_;
  std::unordered_map<std::uint32_t, ActiveContact> active_contacts_;
  std::uint32_t max_contacts_ = 0;
  std::uint32_t max_imus_ = 0;
  bool with_gyro_bias_ = false;
  bool with_unmodeled_wrench_ = false;
  double sampling_time_ = 0.0;
  bool have_stamp_ = false;
  std::int64_t previous_stamp_ns_ = 0;
};

stateObservation::kine::Kinematics toStateObservation(const msg::KineticsKinematics & input);
msg::KineticsKinematics toMessage(const stateObservation::kine::Kinematics & input);
msg::KineticsKinematics toMessage(const stateObservation::kine::LocalKinematics & input);

}  // namespace kinetics_observer_ros2
