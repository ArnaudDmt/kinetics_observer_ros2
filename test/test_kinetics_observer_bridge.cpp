#include <algorithm>
#include <array>
#include <cmath>

#include <gtest/gtest.h>

#include "kinetics_observer_ros2/kinetics_observer_bridge.hpp"

namespace
{

template<class Array>
void identity(Array & values, std::size_t size, double diagonal = 1.0e-6)
{
  std::fill(values.begin(), values.end(), 0.0);
  for(std::size_t index = 0; index < size; ++index) { values[index * size + index] = diagonal; }
}

kinetics_observer_ros2::msg::KineticsConfiguration configuration()
{
  kinetics_observer_ros2::msg::KineticsConfiguration output;
  output.max_contacts = 1;
  output.max_imus = 1;
  output.sampling_time = 0.005;
  output.mass = 50.0;
  output.with_unmodeled_wrench = true;
  output.with_gyro_bias = true;
  output.with_acceleration_estimation = false;
  output.with_damping_in_matrix_a = true;
  output.with_adaptative_contact_process_covariance = true;
  output.use_finite_difference_jacobians = true;
  output.finite_difference_step = 1.0e-6;
  identity(output.state_position_initial_covariance, 3);
  identity(output.state_orientation_initial_covariance, 3);
  identity(output.state_linear_velocity_initial_covariance, 3);
  identity(output.state_angular_velocity_initial_covariance, 3);
  identity(output.gyro_bias_initial_covariance, 3);
  identity(output.unmodeled_wrench_initial_covariance, 6);
  identity(output.contact_initial_covariance, 12);
  identity(output.state_position_process_covariance, 3);
  identity(output.state_orientation_process_covariance, 3);
  identity(output.state_linear_velocity_process_covariance, 3);
  identity(output.state_angular_velocity_process_covariance, 3);
  identity(output.gyro_bias_process_covariance, 3);
  identity(output.unmodeled_wrench_process_covariance, 6);
  identity(output.contact_process_covariance, 12);

  stateObservation::KineticsObserver reference(output.max_contacts, output.max_imus);
  output.initial_state.resize(static_cast<std::size_t>(reference.getStateSize()), 0.0);
  output.initial_state[static_cast<std::size_t>(reference.oriIndex() + 3)] = 1.0;

  kinetics_observer_ros2::msg::KineticsImuConfiguration imu;
  imu.id = 0;
  identity(imu.accelerometer_covariance, 3, 1.0e-4);
  identity(imu.gyroscope_covariance, 3, 1.0e-8);
  output.imus.push_back(imu);

  kinetics_observer_ros2::msg::KineticsContactConfiguration contact;
  contact.id = 0;
  contact.name = "contact";
  contact.has_wrench_sensor = true;
  identity(contact.initial_covariance, 12, 1.0e-4);
  identity(contact.process_covariance, 12, 1.0e-6);
  identity(contact.wrench_covariance, 6, 1.0e-4);
  identity(contact.linear_stiffness, 3, 40000.0);
  identity(contact.linear_damping, 3, 120.0);
  identity(contact.angular_stiffness, 3, 400.0);
  identity(contact.angular_damping, 3, 12.0);
  output.contacts.push_back(contact);
  return output;
}

}  // namespace

TEST(KineticsObserverBridge, RejectsWrongInitialStateSize)
{
  auto config = configuration();
  config.initial_state.pop_back();
  kinetics_observer_ros2::KineticsObserverBridge bridge;
  EXPECT_THROW(bridge.configure(config), std::invalid_argument);
}

TEST(KineticsObserverBridge, ConvertsKinematicsValidityAndQuaternion)
{
  kinetics_observer_ros2::msg::KineticsKinematics input;
  input.valid_fields = input.POSITION | input.ORIENTATION;
  input.position.x = 1.0;
  input.position.y = 2.0;
  input.position.z = 3.0;
  input.orientation.z = std::sin(0.25);
  input.orientation.w = std::cos(0.25);
  const auto converted = kinetics_observer_ros2::toStateObservation(input);
  ASSERT_TRUE(converted.position.isSet());
  ASSERT_TRUE(converted.orientation.isSet());
  ASSERT_FALSE(converted.linVel.isSet());
  const auto round_trip = kinetics_observer_ros2::toMessage(converted);
  EXPECT_EQ(round_trip.valid_fields, input.valid_fields);
  EXPECT_NEAR(round_trip.position.y, 2.0, 1.0e-12);
  EXPECT_NEAR(std::abs(round_trip.orientation.w), std::cos(0.25), 1.0e-12);
}

TEST(KineticsObserverBridge, RunsSamplesAndContactLifecycle)
{
  kinetics_observer_ros2::KineticsObserverBridge bridge;
  bridge.configure(configuration());

  kinetics_observer_ros2::msg::KineticsInput input;
  input.header.stamp.nanosec = 5'000'000;
  input.inertia = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  input.inertia_derivative.fill(0.0);
  kinetics_observer_ros2::msg::KineticsImuInput imu;
  imu.id = 0;
  imu.linear_acceleration.z = 9.81;
  imu.user_imu_kinematics.valid_fields = imu.user_imu_kinematics.ALL;
  imu.user_imu_kinematics.orientation.w = 1.0;
  input.imus.push_back(imu);

  kinetics_observer_ros2::msg::KineticsContactInput contact;
  contact.id = 0;
  contact.active = true;
  contact.initial_world_kinematics.valid_fields =
      contact.initial_world_kinematics.POSITION | contact.initial_world_kinematics.ORIENTATION
      | contact.initial_world_kinematics.LINEAR_VELOCITY | contact.initial_world_kinematics.ANGULAR_VELOCITY;
  contact.initial_world_kinematics.orientation.w = 1.0;
  contact.user_contact_kinematics.valid_fields = contact.user_contact_kinematics.POSITION
                                                 | contact.user_contact_kinematics.ORIENTATION
                                                 | contact.user_contact_kinematics.LINEAR_VELOCITY
                                                 | contact.user_contact_kinematics.ANGULAR_VELOCITY;
  contact.user_contact_kinematics.orientation.w = 1.0;
  contact.measured_wrench.force.z = 50.0 * 9.81;
  input.contacts.push_back(contact);

  const auto output = bridge.update(input);
  EXPECT_FALSE(output.raw_state.empty());
  EXPECT_TRUE(std::all_of(output.raw_state.begin(), output.raw_state.end(), [](double value) {
    return std::isfinite(value);
  }));
  EXPECT_EQ(output.gyro_biases.size(), 1u);
  EXPECT_EQ(output.contacts.size(), 1u);
  EXPECT_TRUE(output.global_floating_base_kinematics.valid_fields
              & output.global_floating_base_kinematics.POSITION);

  // A replay can contain a gap (e.g. one dropped transport sample). The
  // bridge must use the measured interval rather than reject the input.
  input.header.stamp.nanosec = 15'000'000;
  input.contacts.clear();
  const auto without_contact = bridge.update(input);
  EXPECT_TRUE(without_contact.contacts.empty());
  EXPECT_TRUE(std::all_of(without_contact.raw_state.begin(), without_contact.raw_state.end(), [](double value) {
    return std::isfinite(value);
  }));
}

TEST(KineticsObserverBridge, PaperPinContactsAndInitialRestPose)
{
  auto config = configuration();
  config.with_gyro_bias = config.with_unmodeled_wrench = false;
  auto & contact_config = config.contacts.front();
  contact_config.has_wrench_sensor = false;
  contact_config.angular_stiffness.fill(0.0);
  contact_config.angular_damping.fill(0.0);
  contact_config.angular_damping[8] = 12.0;
  contact_config.initial_covariance.fill(0.0);
  contact_config.process_covariance.fill(0.0);
  kinetics_observer_ros2::msg::KineticsInput input;
  input.header.stamp.nanosec = 5'000'000;
  input.inertia = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  kinetics_observer_ros2::msg::KineticsImuInput imu;
  imu.linear_acceleration.z = 9.81;
  imu.user_imu_kinematics.valid_fields = imu.user_imu_kinematics.ALL;
  imu.user_imu_kinematics.orientation.w = 1;
  input.imus.push_back(imu);
  kinetics_observer_ros2::msg::KineticsContactInput contact;
  contact.active = true;
  contact.user_contact_kinematics = imu.user_imu_kinematics;
  input.contacts.push_back(contact);
  for(bool supplied_pose : {false, true})
  {
    auto & initial = input.contacts.front().initial_world_kinematics;
    initial.valid_fields = supplied_pose ? initial.POSITION | initial.ORIENTATION : 0;
    initial.orientation.z = std::sin(M_PI / 12);
    initial.orientation.w = std::cos(M_PI / 12);
    kinetics_observer_ros2::KineticsObserverBridge first, second;
    first.configure(config);
    second.configure(config);
    input.contacts.front().measured_wrench.force.z = 0;
    const auto reference = first.update(input);
    input.contacts.front().measured_wrench.force.z = 1e6;
    input.contacts.front().measured_wrench.torque.x = 1e6;
    const auto output = second.update(input);
    EXPECT_EQ(reference.raw_state, output.raw_state);
    EXPECT_TRUE(std::all_of(output.raw_state.begin(), output.raw_state.end(), [](double v) { return std::isfinite(v); }));
    ASSERT_EQ(output.contacts.size(), 1u);
    EXPECT_NEAR(std::abs(output.contacts.front().rest_kinematics.orientation.z),
                supplied_pose ? std::sin(M_PI / 12) : 0, 1e-9);
  }
}
