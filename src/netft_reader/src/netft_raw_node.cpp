#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "netft_reader/netft_rdt_driver.h"

#include <chrono>
#include <memory>

class NetFTRawNode : public rclcpp::Node
{
public:
  NetFTRawNode()
  : Node("netft_raw_node")
  {
    // Declare parameters
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.1");
    this->declare_parameter<std::string>("frame_id", "ft_sensor");

    std::string ip = this->get_parameter("sensor_ip").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    // Publisher
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data", 10);

    // NetFT driver
    driver_ = std::make_shared<netft_rdt_driver::NetFTRDTDriver>(ip, frame_id);

    // Timer for polling
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100 Hz
      std::bind(&NetFTRawNode::pollSensor, this));

    RCLCPP_INFO(this->get_logger(), "NetFT Raw Node started. Streaming unmodified wrench data.");
  }

private:
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> driver_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void pollSensor()
  {
    if (driver_->waitForNewData()) {
      geometry_msgs::msg::WrenchStamped msg;
      driver_->getData(msg);
      wrench_pub_->publish(msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetFTRawNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
