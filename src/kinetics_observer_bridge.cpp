#include "kinetics_observer_ros2/kinetics_observer_bridge.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <stdexcept>
#include <utility>

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "kinetics_observer_ros2/msg/kinetics_contact_state.hpp"
#include "kinetics_observer_ros2/msg/kinetics_gyro_bias.hpp"

namespace kinetics_observer_ros2
{
namespace
{

using SO = stateObservation::KineticsObserver;
using Kinematics = stateObservation::kine::Kinematics;
using LocalKinematics = stateObservation::kine::LocalKinematics;

stateObservation::Vector3 vector3(const geometry_msgs::msg::Vector3 & value)
{
  if(!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
  {
    throw std::invalid_argument("vector input contains a non-finite value");
  }
  return {value.x, value.y, value.z};
}

stateObservation::Vector3 vector3(const geometry_msgs::msg::Point & value)
{
  if(!std::isfinite(value.x) || !std::isfinite(value.y) || !std::isfinite(value.z))
  {
    throw std::invalid_argument("point input contains a non-finite value");
  }
  return {value.x, value.y, value.z};
}

geometry_msgs::msg::Vector3 vector3Message(const stateObservation::Vector3 & value)
{
  geometry_msgs::msg::Vector3 output;
  output.x = value.x();
  output.y = value.y();
  output.z = value.z();
  return output;
}

geometry_msgs::msg::Point pointMessage(const stateObservation::Vector3 & value)
{
  geometry_msgs::msg::Point output;
  output.x = value.x();
  output.y = value.y();
  output.z = value.z();
  return output;
}

stateObservation::Matrix3 orientation(const geometry_msgs::msg::Quaternion & value)
{
  const double norm = std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z + value.w * value.w);
  if(!std::isfinite(norm) || norm < 1e-12) { throw std::invalid_argument("orientation quaternion is invalid"); }
  const Eigen::Quaterniond quaternion(value.w / norm, value.x / norm, value.y / norm, value.z / norm);
  return quaternion.toRotationMatrix();
}

geometry_msgs::msg::Quaternion orientationMessage(const stateObservation::kine::Orientation & value)
{
  const auto quaternion = value.toQuaternion();
  geometry_msgs::msg::Quaternion output;
  output.x = quaternion.x();
  output.y = quaternion.y();
  output.z = quaternion.z();
  output.w = quaternion.w();
  return output;
}

template<int Rows, int Cols, class Array>
Eigen::Matrix<double, Rows, Cols> matrix(const Array & values, const char * field)
{
  Eigen::Matrix<double, Rows, Cols> output;
  for(int row = 0; row < Rows; ++row)
  {
    for(int column = 0; column < Cols; ++column)
    {
      const double value = values[static_cast<std::size_t>(row * Cols + column)];
      if(!std::isfinite(value)) { throw std::invalid_argument(std::string(field) + " contains a non-finite value"); }
      output(row, column) = value;
    }
  }
  return output;
}

template<int Rows, int Cols, class Array>
void copyMatrix(const Eigen::Matrix<double, Rows, Cols> & input, Array & output)
{
  for(int row = 0; row < Rows; ++row)
  {
    for(int column = 0; column < Cols; ++column)
    {
      output[static_cast<std::size_t>(row * Cols + column)] = input(row, column);
    }
  }
}

stateObservation::Vector6 wrench(const geometry_msgs::msg::Wrench & input)
{
  stateObservation::Vector6 output;
  output << input.force.x, input.force.y, input.force.z, input.torque.x, input.torque.y, input.torque.z;
  return output;
}

geometry_msgs::msg::Wrench wrenchMessage(const stateObservation::Vector6 & input)
{
  geometry_msgs::msg::Wrench output;
  output.force = vector3Message(input.head<3>());
  output.torque = vector3Message(input.tail<3>());
  return output;
}

std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + static_cast<std::int64_t>(stamp.nanosec);
}

template<typename KinematicsType>
msg::KineticsKinematics kinematicsMessage(const KinematicsType & input)
{
  msg::KineticsKinematics output;
  output.valid_fields = 0;
  output.orientation.w = 1.0;
  if(input.position.isSet())
  {
    if(!input.position.getRefUnchecked().allFinite()) { throw std::runtime_error("estimated position is non-finite"); }
    output.valid_fields |= msg::KineticsKinematics::POSITION;
    output.position = pointMessage(input.position.getRefUnchecked());
  }
  if(input.orientation.isSet())
  {
    output.valid_fields |= msg::KineticsKinematics::ORIENTATION;
    output.orientation = orientationMessage(input.orientation);
  }
  if(input.linVel.isSet())
  {
    if(!input.linVel.getRefUnchecked().allFinite())
    {
      throw std::runtime_error("estimated linear velocity is non-finite");
    }
    output.valid_fields |= msg::KineticsKinematics::LINEAR_VELOCITY;
    output.linear_velocity = vector3Message(input.linVel.getRefUnchecked());
  }
  if(input.angVel.isSet())
  {
    if(!input.angVel.getRefUnchecked().allFinite())
    {
      throw std::runtime_error("estimated angular velocity is non-finite");
    }
    output.valid_fields |= msg::KineticsKinematics::ANGULAR_VELOCITY;
    output.angular_velocity = vector3Message(input.angVel.getRefUnchecked());
  }
  if(input.linAcc.isSet())
  {
    if(!input.linAcc.getRefUnchecked().allFinite())
    {
      throw std::runtime_error("estimated linear acceleration is non-finite");
    }
    output.valid_fields |= msg::KineticsKinematics::LINEAR_ACCELERATION;
    output.linear_acceleration = vector3Message(input.linAcc.getRefUnchecked());
  }
  if(input.angAcc.isSet())
  {
    if(!input.angAcc.getRefUnchecked().allFinite())
    {
      throw std::runtime_error("estimated angular acceleration is non-finite");
    }
    output.valid_fields |= msg::KineticsKinematics::ANGULAR_ACCELERATION;
    output.angular_acceleration = vector3Message(input.angAcc.getRefUnchecked());
  }
  return output;
}

}  // namespace

stateObservation::kine::Kinematics toStateObservation(const msg::KineticsKinematics & input)
{
  Kinematics output;
  output.reset();
  const auto valid = input.valid_fields;
  if(valid & msg::KineticsKinematics::POSITION) { output.position = vector3(input.position); }
  if(valid & msg::KineticsKinematics::ORIENTATION) { output.orientation = orientation(input.orientation); }
  if(valid & msg::KineticsKinematics::LINEAR_VELOCITY) { output.linVel = vector3(input.linear_velocity); }
  if(valid & msg::KineticsKinematics::ANGULAR_VELOCITY) { output.angVel = vector3(input.angular_velocity); }
  if(valid & msg::KineticsKinematics::LINEAR_ACCELERATION) { output.linAcc = vector3(input.linear_acceleration); }
  if(valid & msg::KineticsKinematics::ANGULAR_ACCELERATION) { output.angAcc = vector3(input.angular_acceleration); }
  return output;
}

msg::KineticsKinematics toMessage(const Kinematics & input)
{
  return kinematicsMessage(input);
}

msg::KineticsKinematics toMessage(const LocalKinematics & input)
{
  return kinematicsMessage(input);
}

void KineticsObserverBridge::configure(const Configuration & configuration)
{
  if(configuration.max_contacts == 0) { throw std::invalid_argument("max_contacts must be positive"); }
  if(configuration.max_imus == 0) { throw std::invalid_argument("max_imus must be positive"); }
  if(!std::isfinite(configuration.sampling_time) || configuration.sampling_time <= 0.0)
  {
    throw std::invalid_argument("sampling_time must be finite and positive");
  }
  if(!std::isfinite(configuration.mass) || configuration.mass <= 0.0)
  {
    throw std::invalid_argument("mass must be finite and positive");
  }

  auto observer = std::make_unique<SO>(configuration.max_contacts, configuration.max_imus);
  observer->setSamplingTime(configuration.sampling_time);
  observer->setMass(configuration.mass);
  observer->setWithUnmodeledWrench(configuration.with_unmodeled_wrench);
  observer->setWithGyroBias(configuration.with_gyro_bias);
  observer->setWithAccelerationEstimation(configuration.with_acceleration_estimation);
  observer->setWithDampingInMatrixA(configuration.with_damping_in_matrix_a);
  observer->setWithAdaptativeContactProcessCov(configuration.with_adaptative_contact_process_covariance);
  observer->setContactCovLoadWeightExponent(configuration.contact_cov_load_weight_exponent);

  observer->setKinematicsInitCovarianceDefault(
      matrix<3, 3>(configuration.state_position_initial_covariance, "state_position_initial_covariance"),
      matrix<3, 3>(configuration.state_orientation_initial_covariance, "state_orientation_initial_covariance"),
      matrix<3, 3>(configuration.state_linear_velocity_initial_covariance,
                   "state_linear_velocity_initial_covariance"),
      matrix<3, 3>(configuration.state_angular_velocity_initial_covariance,
                   "state_angular_velocity_initial_covariance"));
  observer->setGyroBiasInitCovarianceDefault(
      matrix<3, 3>(configuration.gyro_bias_initial_covariance, "gyro_bias_initial_covariance"));
  observer->setUnmodeledWrenchInitCovMatDefault(
      matrix<6, 6>(configuration.unmodeled_wrench_initial_covariance, "unmodeled_wrench_initial_covariance"));
  observer->setContactInitCovMatDefault(
      matrix<12, 12>(configuration.contact_initial_covariance, "contact_initial_covariance"));

  observer->setKinematicsProcessCovarianceDefault(
      matrix<3, 3>(configuration.state_position_process_covariance, "state_position_process_covariance"),
      matrix<3, 3>(configuration.state_orientation_process_covariance, "state_orientation_process_covariance"),
      matrix<3, 3>(configuration.state_linear_velocity_process_covariance,
                   "state_linear_velocity_process_covariance"),
      matrix<3, 3>(configuration.state_angular_velocity_process_covariance,
                   "state_angular_velocity_process_covariance"));
  observer->setGyroBiasProcessCovarianceDefault(
      matrix<3, 3>(configuration.gyro_bias_process_covariance, "gyro_bias_process_covariance"));
  observer->setUnmodeledWrenchProcessCovarianceDefault(
      matrix<6, 6>(configuration.unmodeled_wrench_process_covariance, "unmodeled_wrench_process_covariance"));
  observer->setContactProcessCovarianceDefault(
      matrix<12, 12>(configuration.contact_process_covariance, "contact_process_covariance"));
  observer->resetStateCovarianceMat();
  observer->resetProcessCovarianceMat();

  if(configuration.use_finite_difference_jacobians)
  {
    if(!std::isfinite(configuration.finite_difference_step) || configuration.finite_difference_step <= 0.0)
    {
      throw std::invalid_argument("finite_difference_step must be finite and positive");
    }
    observer->useFiniteDifferencesJacobians(true);
    stateObservation::Vector step(observer->getStateTangentSize());
    step.setConstant(configuration.finite_difference_step);
    observer->setFiniteDifferenceStep(step);
  }

  if(configuration.initial_state.size() != static_cast<std::size_t>(observer->getStateSize()))
  {
    throw std::invalid_argument("initial_state size does not match configured estimator state size");
  }
  stateObservation::Vector initial_state(observer->getStateSize());
  for(Eigen::Index i = 0; i < initial_state.size(); ++i)
  {
    const double value = configuration.initial_state[static_cast<std::size_t>(i)];
    if(!std::isfinite(value)) { throw std::invalid_argument("initial_state contains a non-finite value"); }
    initial_state[i] = value;
  }
  observer->setInitWorldCentroidStateVector(initial_state);

  std::unordered_map<std::uint32_t, msg::KineticsImuConfiguration> imu_configurations;
  for(const auto & imu : configuration.imus)
  {
    if(imu.id >= configuration.max_imus) { throw std::invalid_argument("configured IMU id exceeds max_imus"); }
    if(!imu_configurations.emplace(imu.id, imu).second) { throw std::invalid_argument("duplicate configured IMU id"); }
    matrix<3, 3>(imu.accelerometer_covariance, "accelerometer_covariance");
    matrix<3, 3>(imu.gyroscope_covariance, "gyroscope_covariance");
  }
  std::unordered_map<std::uint32_t, msg::KineticsContactConfiguration> contact_configurations;
  for(const auto & contact : configuration.contacts)
  {
    if(contact.id >= configuration.max_contacts)
    {
      throw std::invalid_argument("configured contact id exceeds max_contacts");
    }
    if(contact.name.empty()) { throw std::invalid_argument("configured contact name must not be empty"); }
    if(!contact_configurations.emplace(contact.id, contact).second)
    {
      throw std::invalid_argument("duplicate configured contact id");
    }
  }

  observer_ = std::move(observer);
  imu_configurations_ = std::move(imu_configurations);
  contact_configurations_ = std::move(contact_configurations);
  max_contacts_ = configuration.max_contacts;
  max_imus_ = configuration.max_imus;
  with_gyro_bias_ = configuration.with_gyro_bias;
  with_unmodeled_wrench_ = configuration.with_unmodeled_wrench;
  sampling_time_ = configuration.sampling_time;
  active_contacts_.clear();
  have_stamp_ = false;
}

KineticsObserverBridge::State KineticsObserverBridge::update(const Input & input)
{
  if(!observer_) { throw std::logic_error("observer is not configured"); }
  const auto stamp_ns = stampNanoseconds(input.header.stamp);
  if(have_stamp_ && stamp_ns <= previous_stamp_ns_) { throw std::invalid_argument("input timestamps must increase"); }

  observer_->setCenterOfMass(vector3(input.center_of_mass), vector3(input.center_of_mass_velocity),
                             vector3(input.center_of_mass_acceleration));
  observer_->setCoMAngularMomentum(vector3(input.angular_momentum), vector3(input.angular_momentum_derivative));
  observer_->setCoMInertiaMatrix(matrix<3, 3>(input.inertia, "inertia"),
                                 matrix<3, 3>(input.inertia_derivative, "inertia_derivative"));

  std::set<std::uint32_t> imu_ids;
  for(const auto & imu : input.imus)
  {
    if(imu.id >= max_imus_) { throw std::invalid_argument("IMU id exceeds max_imus"); }
    if(!imu_ids.insert(imu.id).second) { throw std::invalid_argument("duplicate IMU id"); }
    const auto configured = imu_configurations_.find(imu.id);
    if(configured == imu_configurations_.end()) { throw std::invalid_argument("IMU id has no configuration"); }
    observer_->setIMU(vector3(imu.linear_acceleration), vector3(imu.angular_velocity),
                      matrix<3, 3>(configured->second.accelerometer_covariance, "accelerometer_covariance"),
                      matrix<3, 3>(configured->second.gyroscope_covariance, "gyroscope_covariance"),
                      toStateObservation(imu.user_imu_kinematics), static_cast<stateObservation::Index>(imu.id));
  }

  std::set<std::uint32_t> contact_ids;
  std::set<std::uint32_t> next_active;
  const bool had_active_contacts = !active_contacts_.empty();
  for(const auto & contact : input.contacts)
  {
    if(contact.id >= max_contacts_) { throw std::invalid_argument("contact id exceeds max_contacts"); }
    if(!contact_ids.insert(contact.id).second) { throw std::invalid_argument("duplicate contact id"); }
    if(contact.active) { next_active.insert(contact.id); }
  }

  for(auto iterator = active_contacts_.begin(); iterator != active_contacts_.end();)
  {
    if(next_active.count(iterator->first) == 0)
    {
      observer_->removeContact(static_cast<stateObservation::Index>(iterator->first));
      iterator = active_contacts_.erase(iterator);
    }
    else { ++iterator; }
  }

  for(const auto & contact : input.contacts)
  {
    if(!contact.active) { continue; }
    const auto configured = contact_configurations_.find(contact.id);
    if(configured == contact_configurations_.end()) { throw std::invalid_argument("contact id has no configuration"); }
    const auto & contact_configuration = configured->second;
    auto active = active_contacts_.find(contact.id);

    // The measured wrench is rotated into the contact frame the observer assumes. A calibration
    // tilt leaks normal load into the tangential axes: on RHPS1 each foot reads 0.29x its normal
    // force sideways, in a direction fixed to that foot, which statics rules out during single
    // support. Left uncorrected the unmodeled wrench absorbs the resulting ~147 N, so it is no
    // longer free to represent genuine unmodeled dynamics. Zero leaves the wrench untouched.
    auto measured_wrench = wrench(contact.measured_wrench);
    {
      const Eigen::Map<const stateObservation::Vector3> tilt(contact_configuration.wrench_tilt_correction.data());
      const double angle = tilt.norm();
      if(std::isfinite(angle) && angle > 0.0)
      {
        const stateObservation::Matrix3 rotation(Eigen::AngleAxisd(angle, tilt / angle));
        // Force only. A sensor-frame rotation would tilt the torque equally, but the centre of
        // pressure is already sensible before correction (well inside the sole) and this rotation
        // moves it under 1.5 mm, so the torque carries no matching defect and is left alone.
        measured_wrench.head<3>() = rotation * measured_wrench.head<3>();
      }
    }
    const auto user_kinematics = toStateObservation(contact.user_contact_kinematics);
    if(active == active_contacts_.end())
    {
      const auto linear_stiffness = matrix<3, 3>(contact_configuration.linear_stiffness, "contact.linear_stiffness");
      const auto linear_damping = matrix<3, 3>(contact_configuration.linear_damping, "contact.linear_damping");
      const auto angular_stiffness = matrix<3, 3>(contact_configuration.angular_stiffness, "contact.angular_stiffness");
      const auto angular_damping = matrix<3, 3>(contact_configuration.angular_damping, "contact.angular_damping");
      auto world_contact = observer_->getGlobalKinematicsOf(user_kinematics);
      observer_->addContact(
          world_contact,
          matrix<12, 12>(had_active_contacts ? contact_configuration.new_contact_initial_covariance
                                              : contact_configuration.initial_covariance,
                         "contact.initial_covariance"),
          matrix<12, 12>(contact_configuration.process_covariance, "contact.process_covariance"),
          static_cast<stateObservation::Index>(contact.id), linear_stiffness, linear_damping, angular_stiffness,
          angular_damping, measured_wrench.head<3>(), measured_wrench.tail<3>(), contact_configuration.flat_odometry);
      active_contacts_.emplace(contact.id, ActiveContact{contact_configuration.name});
    }

    if(contact_configuration.has_wrench_sensor)
    {
      const auto usable = [](const auto & values)
      { return std::any_of(values.begin(), values.end(),
                           [](double value) { return std::isfinite(value) && value != 0.0; }); };
      stateObservation::Matrix6 covariance;
      if(usable(contact.wrench_covariance_transform))
      {
        // The log carries the sensor->contact wrench transform, so the configured covariance can be
        // transported here rather than baked in upstream. This keeps contact_wrench tunable per replay.
        const auto transform =
            matrix<6, 6>(contact.wrench_covariance_transform, "contact.wrench_covariance_transform");
        covariance = transform
                     * matrix<6, 6>(contact_configuration.wrench_covariance, "contact.wrench_covariance")
                     * transform.transpose();
      }
      else if(usable(contact.wrench_covariance))
      {
        covariance = matrix<6, 6>(contact.wrench_covariance, "contact.wrench_covariance");
      }
      else { covariance = matrix<6, 6>(contact_configuration.wrench_covariance, "contact.wrench_covariance"); }
      observer_->updateContactWithWrenchSensor(measured_wrench, covariance, user_kinematics, contact.id);
    }
    else { observer_->updateContactWithNoSensor(user_kinematics, contact.id); }
  }

  observer_->setAdditionalWrench(vector3(input.additional_wrench.force), vector3(input.additional_wrench.torque));
  const auto & state = observer_->update();
  if(!state.allFinite())
  {
    for(Eigen::Index index = 0; index < state.size(); ++index)
    {
      if(!std::isfinite(state[index]))
      {
        throw std::runtime_error("estimated state contains a non-finite value at index " + std::to_string(index));
      }
    }
  }

  State output;
  output.header = input.header;
  output.global_centroid_kinematics = toMessage(observer_->getGlobalCentroidKinematics());
  output.local_centroid_kinematics = toMessage(observer_->getLocalCentroidKinematics());
  Kinematics floating_base;
  floating_base.setZero<stateObservation::Matrix3>(Kinematics::Flags::all);
  output.global_floating_base_kinematics = toMessage(observer_->getGlobalKinematicsOf(floating_base));
  output.raw_state.assign(state.data(), state.data() + state.size());

  if(with_unmodeled_wrench_)
  {
    output.unmodeled_wrench =
        wrenchMessage(state.segment<6>(static_cast<Eigen::Index>(observer_->unmodeledWrenchIndex())));
  }
  for(std::uint32_t id = 0; with_gyro_bias_ && id < max_imus_; ++id)
  {
    msg::KineticsGyroBias bias;
    bias.id = id;
    bias.bias = vector3Message(
        state.segment<3>(static_cast<Eigen::Index>(observer_->gyroBiasIndex(static_cast<stateObservation::Index>(id)))));
    output.gyro_biases.push_back(bias);
  }
  for(const auto id_value : observer_->getListOfContacts())
  {
    const auto id = static_cast<std::uint32_t>(id_value);
    msg::KineticsContactState contact;
    contact.id = id;
    contact.name = active_contacts_.at(id).name;
    contact.rest_kinematics = toMessage(observer_->getContactStateRestKinematics(id_value));
    stateObservation::Vector6 state_wrench;
    state_wrench.head<3>() =
        state.segment<3>(static_cast<Eigen::Index>(observer_->contactForceIndex(id_value)));
    state_wrench.tail<3>() =
        state.segment<3>(static_cast<Eigen::Index>(observer_->contactTorqueIndex(id_value)));
    contact.state_wrench = wrenchMessage(state_wrench);
    contact.viscoelastic_wrench = wrenchMessage(observer_->getCurrentViscoElasticWrench(id_value));
    output.contacts.push_back(contact);
  }

  have_stamp_ = true;
  previous_stamp_ns_ = stamp_ns;
  return output;
}

}  // namespace kinetics_observer_ros2
