#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "CombinedApi.h"
#include "ToolData.h"
#include "PortHandleInfo.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <iostream>
#include <fstream>
#include <iomanip>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

class NDIRelativePoseNode : public rclcpp::Node {
public:
  NDIRelativePoseNode()
  : Node("ndi_relative_pose_node"), file_index_(0),
    has_initial_pose_(false), tracking_(false), pending_reset_(false)
  {
    this->declare_parameter<std::string>("device", "/dev/ttyUSB0");
    this->declare_parameter<std::string>("topic", "/sensor_pose");
    this->declare_parameter<int>("tool_index", 0);

    std::string device = this->get_parameter("device").as_string();
    std::string topic = this->get_parameter("topic").as_string();
    int tool_index = this->get_parameter("tool_index").as_int();

    if (capi_.connect(device, Protocol::TCP) != 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to connect to %s", device.c_str());
      rclcpp::shutdown();
      return;
    }

    capi_.initialize();
    determineBX2Support();
    sleep(1);
    initializeAndEnableTools();

    if (enabledTools_.size() <= tool_index) {
      RCLCPP_FATAL(this->get_logger(), "Tool index %d out of range.", tool_index);
      rclcpp::shutdown();
      return;
    }

    if (capi_.startTracking() < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to start tracking.");
      rclcpp::shutdown();
      return;
    }

    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>(topic, 10);

    zero_pose_service_ = this->create_service<std_srvs::srv::Trigger>(
      "set_zero_pose",
      std::bind(&NDIRelativePoseNode::handleZeroPoseService, this, std::placeholders::_1, std::placeholders::_2));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      [this, tool_index]() {
        auto tools = apiSupportsBX2_ ? capi_.getTrackingDataBX2() : capi_.getTrackingDataBX();
        if (tool_index >= tools.size() || tools[tool_index].transform.isMissing()) {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Tool missing.");
          return;
        }

        auto &t = tools[tool_index].transform;
        Eigen::Vector3d position(t.tx, t.ty, t.tz);
        Eigen::Vector3d tangent(t.qx, t.qy, t.q0);
        tangent.normalize();

        geometry_msgs::msg::Pose msg;

        if (!has_initial_pose_ && pending_reset_) {
          initial_position_ = position;
          initial_tangent_ = tangent;
          has_initial_pose_ = true;
          pending_reset_ = false;
          openNewLogFile();
          tracking_ = true;
          RCLCPP_INFO(this->get_logger(), "Initial pose set. Tracking started.");
          return;
        }

        if (has_initial_pose_) {
          Eigen::Vector3d delta_pos = position - initial_position_;
          msg.position.x = delta_pos.x();
          msg.position.y = delta_pos.y();
          msg.position.z = delta_pos.z();

          msg.orientation.x = tangent.x();
          msg.orientation.y = tangent.y();
          msg.orientation.z = 0.0;
          msg.orientation.w = tangent.z();

          publisher_->publish(msg);

          if (tracking_) {
            auto now = this->get_clock()->now();
            logfile_ << std::fixed << std::setprecision(6)
                     << now.seconds() << ","
                     << delta_pos.x() << "," << delta_pos.y() << "," << delta_pos.z() << ","
                     << tangent.x() << "," << tangent.y() << "," << tangent.z() << "\n";

            RCLCPP_INFO(this->get_logger(), "ΔPos: [%.3f %.3f %.3f] Tangent: [%.3f %.3f %.3f]",
                        delta_pos.x(), delta_pos.y(), delta_pos.z(),
                        tangent.x(), tangent.y(), tangent.z());
          }
        }
      });

    std::thread(&NDIRelativePoseNode::keyboardLoop, this).detach();
    RCLCPP_INFO(this->get_logger(), "NDI node initialized. Press 'r' or call /set_zero_pose.");
  }

  ~NDIRelativePoseNode() {
    if (logfile_.is_open())
      logfile_.close();
  }

private:
  CombinedApi capi_;
  bool apiSupportsBX2_ = false;
  std::vector<ToolData> enabledTools_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_pose_service_;

  std::ofstream logfile_;
  std::atomic<bool> has_initial_pose_;
  std::atomic<bool> tracking_;
  std::atomic<bool> pending_reset_;
  std::atomic<int> file_index_;
  Eigen::Vector3d initial_position_;
  Eigen::Vector3d initial_tangent_;

  void determineBX2Support() {
    std::string rev = capi_.getApiRevision();
    if (rev[0] == 'G' && capi_.stringToInt(rev.substr(2, 3)) >= 3)
      apiSupportsBX2_ = true;
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
      td.transform.toolHandle = static_cast<uint16_t>(capi_.stringToInt(ph.getPortHandle()));
      enabledTools_.push_back(td);
    }
  }

  void keyboardLoop() {
    while (rclcpp::ok()) {
      char c = getcharNonblock();
      if (c == 'r') {
        RCLCPP_INFO(this->get_logger(), "Keyboard: reset requested. Will zero on next update.");
        tracking_ = false;
        has_initial_pose_ = false;
        pending_reset_ = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void handleZeroPoseService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service: reset requested. Will zero on next update.");
    tracking_ = false;
    has_initial_pose_ = false;
    pending_reset_ = true;

    response->success = true;
    response->message = "Pose reset triggered.";
  }

  void openNewLogFile() {
    if (logfile_.is_open()) logfile_.close();

    std::ostringstream fname;
    fname << "relative_pose_" << file_index_++ << ".csv";
    logfile_.open(fname.str(), std::ios::out | std::ios::trunc);
    logfile_ << "time_sec,dx,dy,dz,tangent_x,tangent_y,tangent_z\n";
  }

  char getcharNonblock() {
    struct termios oldt, newt;
    char ch = 0;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NDIRelativePoseNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
