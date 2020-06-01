// STL
#include <chrono>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

// ROS
#include <rclcpp/rclcpp.hpp>

// msg
#include <rmf_lift_msgs/msg/lift_request.hpp>

void print_instructions()
{
  std::cout << "Invalid number of arguments, please pass in lift_name, desired level and door state"
            << " after the binary in that order, only supports 'open' or 'closed'." << std::endl;
  std::cout << "For example, \n"
            << "  request_lift Lift1 L1 open\n"
            << "  request_lift Lift2 L4 closed\n" << std::endl;
}

int main(int argc, char* argv[])
{
  // handle the request through command line
  if (argc != 4) {
    print_instructions();
    return 1;
  }
  std::string lift_name = argv[1];
  std::string floor_request = argv[2];
  std::string door_request = argv[3];

  rclcpp::init(argc, argv);

  using LiftRequest = rmf_lift_msgs::msg::LiftRequest;
  auto node = rclcpp::Node::make_shared("lift_resuest_publisher");
  auto publisher = node->create_publisher<LiftRequest>("/lift_requests", rclcpp::SystemDefaultsQoS());
  auto msg = std::make_shared<LiftRequest>();

  msg->lift_name = lift_name;
  msg->destination_floor = floor_request;

  if (door_request == "open")
  {
    msg->door_state = static_cast<uint8_t>(LiftRequest::DOOR_OPEN);
  }
  else if (door_request == "closed")
  {
    msg->door_state = static_cast<uint8_t>(LiftRequest::DOOR_CLOSED);
  }
  else
  {
    print_instructions();
    return 1;
  }

  if (rclcpp::ok())
  {
    for (int i = 0; i < 5; i++)
    {
      publisher->publish(*msg);
      rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    std::cout << "Sent 5 messages at 2 Hz requesting lift: " << lift_name;
    std::cout << ", to floor: " << floor_request;
    std::cout << ", with door: " << door_request << std::endl;
    return 0;
  }
  return 1;
}