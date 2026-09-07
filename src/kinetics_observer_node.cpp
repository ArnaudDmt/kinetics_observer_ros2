#include <deque>
#include <memory>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>

#include "kinetics_observer_ros2/kinetics_observer_bridge.hpp"

namespace kinetics_observer_ros2
{

class KineticsObserverNode : public rclcpp::Node
{
public:
  KineticsObserverNode() : Node("kinetics_observer")
  {
    configuration_subscription_ = create_subscription<msg::KineticsConfiguration>(
        "~/configuration", rclcpp::QoS(1).reliable().transient_local(),
        [this](const msg::KineticsConfiguration::SharedPtr message)
        {
          try
          {
            bridge_.configure(*message);
            configured_ = true;
            RCLCPP_INFO(get_logger(), "Kinetics Observer configured (%u contacts, %u IMUs).", message->max_contacts,
                        message->max_imus);
            for(const auto & input : pending_inputs_) { process_input(input); }
            pending_inputs_.clear();
          }
          catch(const std::exception & error)
          {
            RCLCPP_ERROR(get_logger(), "Rejected configuration: %s", error.what());
          }
        });

    input_subscription_ = create_subscription<msg::KineticsInput>(
        "~/input", rclcpp::QoS(rclcpp::KeepAll()).reliable(),
        [this](const msg::KineticsInput::SharedPtr message)
        {
          if(!configured_) { pending_inputs_.push_back(message); }
          else { process_input(message); }
        });

    state_publisher_ = create_publisher<msg::KineticsState>("~/estimated_state", rclcpp::QoS(10).reliable());
    RCLCPP_INFO(get_logger(), "Kinetics Observer ready; waiting for ~/configuration.");
  }

private:
  void process_input(const msg::KineticsInput::SharedPtr & message)
  {
    try
    {
      state_publisher_->publish(bridge_.update(*message));
    }
    catch(const std::exception & error)
    {
      RCLCPP_ERROR(get_logger(), "Rejected input: %s", error.what());
    }
  }

  KineticsObserverBridge bridge_;
  std::deque<msg::KineticsInput::SharedPtr> pending_inputs_;
  bool configured_ = false;
  rclcpp::Subscription<msg::KineticsConfiguration>::SharedPtr configuration_subscription_;
  rclcpp::Subscription<msg::KineticsInput>::SharedPtr input_subscription_;
  rclcpp::Publisher<msg::KineticsState>::SharedPtr state_publisher_;
};

}  // namespace kinetics_observer_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kinetics_observer_ros2::KineticsObserverNode>());
  rclcpp::shutdown();
  return 0;
}
