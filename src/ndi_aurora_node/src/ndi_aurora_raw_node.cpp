#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"   // <<< stamped
#include "CombinedApi.h"
#include "ToolData.h"
#include "PortHandleInfo.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <thread>
#include <chrono>

class NDIRawPoseNode : public rclcpp::Node {
public:
  NDIRawPoseNode()
  : Node("ndi_aurora_raw_node")
  {
    // parameters
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("topic", "/sensor_pose_raw");
    this->declare_parameter<std::string>("frame_id", "ndi");
    this->declare_parameter<int>("tool_index", 0);

    device_    = this->get_parameter("device").as_string();
    topic_     = this->get_parameter("topic").as_string();
    frame_id_  = this->get_parameter("frame_id").as_string();
    tool_index_= this->get_parameter("tool_index").as_int();

    if (capi_.connect(device_, Protocol::TCP) != 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to %s", device_.c_str());
      rclcpp::shutdown();
      return;
    }

    capi_.initialize();
    determineBX2Support();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    initializeAndEnableTools();

    if (tool_index_ >= enabledTools_.size()) {
      RCLCPP_FATAL(this->get_logger(), "Tool index %d out of range.", tool_index_);
      rclcpp::shutdown();
      return;
    }

    if (capi_.startTracking() < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start tracking.");
      rclcpp::shutdown();
      return;
    }

    // <<< pose_stamped publisher
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(topic_, 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(25),
      [this]() {
        auto tools = apiSupportsBX2_
          ? capi_.getTrackingDataBX2()
          : capi_.getTrackingDataBX();
        if (tool_index_ >= tools.size() || tools[tool_index_].transform.isMissing()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(),
            *this->get_clock(), 2000, "Tool missing.");
          return;
        }

        auto &t = tools[tool_index_].transform;
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();              // stamp it
        msg.header.frame_id = frame_id_;             // frame

        msg.pose.position.x    = t.tx;
        msg.pose.position.y    = t.ty;
        msg.pose.position.z    = t.tz;
        msg.pose.orientation.x = t.qx;
        msg.pose.orientation.y = t.qy;
        msg.pose.orientation.z = t.qz;
        msg.pose.orientation.w = t.q0;

        publisher_->publish(msg);
      });

    RCLCPP_INFO(this->get_logger(), "NDI raw pose node initialized.");
  }

private:
  // NDI API
  CombinedApi capi_;
  bool apiSupportsBX2_{false};
  std::vector<ToolData> enabledTools_;

  // ROS
  std::string device_, topic_, frame_id_;
  int tool_index_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void determineBX2Support() {
    auto rev = capi_.getApiRevision();
    apiSupportsBX2_ = (rev[0]=='G'
      && capi_.stringToInt(rev.substr(2,3)) >= 3);
  }

  void initializeAndEnableTools() {
    auto handles = capi_.portHandleSearchRequest(PortHandleSearchRequestOption::NotInit);
    for (auto &ph : handles) {
      capi_.portHandleInitialize(ph.getPortHandle());
      capi_.portHandleEnable(ph.getPortHandle());
    }
    auto enabled = capi_.portHandleSearchRequest(PortHandleSearchRequestOption::Enabled);
    for (auto &ph : enabled) {
      ToolData td;
      td.transform.toolHandle = static_cast<uint16_t>(
        capi_.stringToInt(ph.getPortHandle()));
      enabledTools_.push_back(td);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NDIRawPoseNode>());
  rclcpp::shutdown();
  return 0;
}
