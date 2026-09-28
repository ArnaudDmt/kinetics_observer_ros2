#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include "kinetics_observer_ros2/kinetics_observer_bridge.hpp"

namespace
{
constexpr auto configuration_topic = "/kinetics_observer/configuration";
constexpr auto input_topic = "/kinetics_observer/input";

template<typename Message>
Message deserialize(const std::shared_ptr<rosbag2_storage::SerializedBagMessage> & bag_message)
{
  rclcpp::SerializedMessage serialized(*bag_message->serialized_data);
  rclcpp::Serialization<Message> serialization;
  Message output;
  serialization.deserialize_message(&serialized, &output);
  return output;
}

kinetics_observer_ros2::msg::KineticsConfiguration readConfiguration(const std::string & bag)
{
  rosbag2_cpp::Reader reader;
  reader.open(bag);
  while(reader.has_next())
  {
    auto message = reader.read_next();
    if(message->topic_name == configuration_topic)
    {
      return deserialize<kinetics_observer_ros2::msg::KineticsConfiguration>(message);
    }
  }
  throw std::runtime_error("configuration bag has no " + std::string(configuration_topic));
}

double seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

void writePose(std::ofstream & output, double stamp, const kinetics_observer_ros2::msg::KineticsKinematics & pose)
{
  const auto required = pose.POSITION | pose.ORIENTATION;
  if((pose.valid_fields & required) != required) { throw std::runtime_error("floating-base pose is incomplete"); }
  output << std::setprecision(17) << stamp << ' ' << pose.position.x << ' ' << pose.position.y << ' '
         << pose.position.z << ' ' << pose.orientation.x << ' ' << pose.orientation.y << ' ' << pose.orientation.z
         << ' ' << pose.orientation.w << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    if(argc != 4 && argc != 5)
    {
      throw std::invalid_argument(
          "usage: kinetics_offline_replay INPUT_BAG CONFIGURATION_BAG OUTPUT_TRAJECTORY [STATE_DUMP]");
    }
    kinetics_observer_ros2::KineticsObserverBridge bridge;
    bridge.configure(readConfiguration(argv[2]));
    std::ofstream output(argv[3]);
    if(!output) { throw std::runtime_error("cannot open output trajectory: " + std::string(argv[3])); }
    output << "# timestamp tx ty tz qx qy qz qw\n";

    std::ofstream dump;
    if(argc == 5)
    {
      dump.open(argv[4]);
      if(!dump) { throw std::runtime_error("cannot open state dump: " + std::string(argv[4])); }
      dump << "# timestamp field... (one line per tick)\n";
    }

    rosbag2_cpp::Reader reader;
    reader.open(argv[1]);
    std::size_t skip = 0;
    if(const char * env = std::getenv("KO_SKIP_INPUTS")) { skip = std::stoul(env); }
    std::size_t skipped = 0;
    std::size_t count = 0;
    double first_stamp = 0.0;
    double last_stamp = 0.0;
    const auto started = std::chrono::steady_clock::now();
    while(reader.has_next())
    {
      auto bag_message = reader.read_next();
      if(bag_message->topic_name != input_topic) { continue; }
      if(skipped < skip) { ++skipped; continue; }
      const auto input = deserialize<kinetics_observer_ros2::msg::KineticsInput>(bag_message);
      const double stamp = seconds(input.header.stamp);
      const auto state = bridge.update(input);
      writePose(output, stamp, state.global_floating_base_kinematics);
      if(dump.is_open())
      {
        dump << std::setprecision(17) << stamp;
        const auto & c = state.global_centroid_kinematics;
        dump << " centroid " << c.position.x << ' ' << c.position.y << ' ' << c.position.z
             << ' ' << c.orientation.x << ' ' << c.orientation.y << ' ' << c.orientation.z << ' '
             << c.orientation.w << ' ' << c.linear_velocity.x << ' ' << c.linear_velocity.y << ' '
             << c.linear_velocity.z << ' ' << c.angular_velocity.x << ' ' << c.angular_velocity.y
             << ' ' << c.angular_velocity.z;
        dump << " unmodeled " << state.unmodeled_wrench.force.x << ' ' << state.unmodeled_wrench.force.y
             << ' ' << state.unmodeled_wrench.force.z << ' ' << state.unmodeled_wrench.torque.x << ' '
             << state.unmodeled_wrench.torque.y << ' ' << state.unmodeled_wrench.torque.z;
        for(const auto & bias : state.gyro_biases)
        {
          dump << " gyrobias " << bias.id << ' ' << bias.bias.x << ' ' << bias.bias.y << ' ' << bias.bias.z;
        }
        for(const auto & contact : state.contacts)
        {
          const auto & r = contact.rest_kinematics;
          dump << " contact " << contact.name << ' ' << r.position.x << ' ' << r.position.y << ' '
               << r.position.z << ' ' << r.orientation.x << ' ' << r.orientation.y << ' '
               << r.orientation.z << ' ' << r.orientation.w << ' ' << contact.state_wrench.force.x << ' '
               << contact.state_wrench.force.y << ' ' << contact.state_wrench.force.z << ' '
               << contact.state_wrench.torque.x << ' ' << contact.state_wrench.torque.y << ' '
               << contact.state_wrench.torque.z;
        }
        dump << '\n';
      }
      if(count++ == 0) { first_stamp = stamp; }
      last_stamp = stamp;
    }
    if(count == 0) { throw std::runtime_error("input bag has no " + std::string(input_topic)); }

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    const double duration = last_stamp - first_stamp;
    std::cout << "Processed " << count << " samples in " << elapsed << " s";
    if(elapsed > 0.0 && duration > 0.0) { std::cout << " (" << duration / elapsed << "x real time)"; }
    std::cout << std::endl;
    rclcpp::shutdown();
    return 0;
  }
  catch(const std::exception & error)
  {
    std::cerr << "kinetics_offline_replay: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 2;
  }
}
