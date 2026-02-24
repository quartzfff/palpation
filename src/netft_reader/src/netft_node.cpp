#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "netft_reader/netft_rdt_driver.h"

#include <fstream>
#include <atomic>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <iomanip>

class NetFTNode : public rclcpp::Node
{
public:
  NetFTNode()
  : Node("netft_node"), recording_(false), file_index_(0)
  {
    // Parameters
    this->declare_parameter<std::string>("sensor_ip", "192.168.1.1");
    this->declare_parameter<std::string>("frame_id", "ft_sensor");

    std::string ip = this->get_parameter("sensor_ip").as_string();
    std::string frame_id = this->get_parameter("frame_id").as_string();

    // Publishers
    wrench_pub_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>("netft_data", 10);
    ready_pub_  = this->create_publisher<std_msgs::msg::Bool>("netft_ready", 1);

    // Service
    zero_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "zero_force",
      std::bind(&NetFTNode::handleZeroService, this, std::placeholders::_1, std::placeholders::_2));

    // Driver
    driver_ = std::make_shared<netft_rdt_driver::NetFTRDTDriver>(ip, frame_id);

    // Initial zero
    RCLCPP_INFO(this->get_logger(), "Waiting 1 second before zeroing...");
    rclcpp::sleep_for(std::chrono::seconds(1));
    driver_->zero();
    RCLCPP_INFO(this->get_logger(), "Sensor zeroed. Press 'z' to zero again and start logging.");

    // Keyboard thread
    std::thread(&NetFTNode::keyboardLoop, this).detach();

    // Timer for main loop
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 500 Hz
      std::bind(&NetFTNode::timerCallback, this));
  }

  ~NetFTNode()
  {
    if (logfile_.is_open())
      logfile_.close();
  }

private:
  std::shared_ptr<netft_rdt_driver::NetFTRDTDriver> driver_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr zero_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::ofstream logfile_;
  std::atomic<bool> recording_;
  std::atomic<int> file_index_;
  std_msgs::msg::Bool is_ready_msg_;

  void timerCallback()
  {
    if (driver_->waitForNewData()) {
      geometry_msgs::msg::WrenchStamped msg;
      driver_->getData(msg);
      wrench_pub_->publish(msg);

      is_ready_msg_.data = true;
      ready_pub_->publish(is_ready_msg_);

      if (recording_ && logfile_.is_open()) {
        auto now = this->get_clock()->now();
        logfile_ << std::fixed << std::setprecision(6)
                 << now.seconds() << ","
                 << msg.wrench.force.x << ","
                 << msg.wrench.force.y << ","
                 << msg.wrench.force.z << ","
                 << msg.wrench.torque.x << ","
                 << msg.wrench.torque.y << ","
                 << msg.wrench.torque.z << "\n";
      }
    }
  }

  void handleZeroService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Service: zeroing sensor and starting new log...");
    driver_->zero();
    openNewLogFile();
    recording_ = true;
    response->success = true;
    response->message = "Sensor zeroed and logging started.";
  }

  void openNewLogFile()
  {
    if (logfile_.is_open())
      logfile_.close();

    std::ostringstream fname;
    fname << "force_log_" << file_index_++ << ".csv";
    logfile_.open(fname.str(), std::ios::out | std::ios::trunc);
    logfile_ << "time_sec,fx,fy,fz,tx,ty,tz\n";

    RCLCPP_INFO(this->get_logger(), "Logging to %s", fname.str().c_str());
  }

  void keyboardLoop()
  {
    RCLCPP_INFO(this->get_logger(), "Press 'z' to zero and log new CSV");

    while (rclcpp::ok()) {
      char c = getcharNonblock();
      if (c == 'z') {
        RCLCPP_INFO(this->get_logger(), "Keyboard: zeroing sensor and starting new log...");
        driver_->zero();
        openNewLogFile();
        recording_ = true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  char getcharNonblock()
  {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NetFTNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
