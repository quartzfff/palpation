// src/sync_logger_node.cpp

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <mutex>
#include <vector>
#include <sstream>

class SyncLoggerNode : public rclcpp::Node
{
public:
  SyncLoggerNode()
  : Node("sync_logger_node"),
    recording_(false),
    collecting_reference_(false),
    init_count_(0),
    file_index_(0)
  {
    using WrenchStamped = geometry_msgs::msg::WrenchStamped;
    using PoseStamped   = geometry_msgs::msg::PoseStamped;
    using SyncPolicy    = message_filters::sync_policies::ApproximateTime<WrenchStamped,PoseStamped>;

    // wrap topics in message_filters subscribers
    wrench_sub_ = std::make_shared<message_filters::Subscriber<WrenchStamped>>(this, "/netft_data");
    pose_sub_   = std::make_shared<message_filters::Subscriber<PoseStamped>>( this, "/sensor_pose_raw");

    // approximate‐time synchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
      SyncPolicy(10), *wrench_sub_, *pose_sub_);
    sync_->registerCallback(
      std::bind(&SyncLoggerNode::onBoth, this, std::placeholders::_1, std::placeholders::_2)
    );

    // keyboard thread for “r” press
    std::thread(&SyncLoggerNode::keyboardLoop, this).detach();

    RCLCPP_INFO(get_logger(),
      "Press ‘r’ to collect 10 reference points, then start synchronized logging.");
  }

  ~SyncLoggerNode() override {
    if (csv_.is_open()) csv_.close();
  }

private:
  using WrenchStamped = geometry_msgs::msg::WrenchStamped;
  using PoseStamped   = geometry_msgs::msg::PoseStamped;

  // message_filters subscribers + synchronizer
  std::shared_ptr<message_filters::Subscriber<WrenchStamped>> wrench_sub_;
  std::shared_ptr<message_filters::Subscriber<PoseStamped>>   pose_sub_;
  std::shared_ptr<message_filters::Synchronizer<
      message_filters::sync_policies::ApproximateTime<WrenchStamped,PoseStamped>>> sync_;

  // Latest msgs
  WrenchStamped latest_wrench_;
  PoseStamped   latest_pose_;

  // zero‐offset accumulators
  Eigen::Vector3d zero_f_{0,0,0}, zero_t_{0,0,0}, zero_p_{0,0,0};
  std::vector<Eigen::Vector3d> fs_, ts_, ps_;
  int init_count_;
  bool collecting_reference_;

  // CSV logging
  std::mutex      mtx_;
  std::ofstream   csv_;
  std::atomic<bool> recording_;
  int file_index_;

  // Called on each synchronized pair
  void onBoth(
    const WrenchStamped::ConstSharedPtr &w,
    const PoseStamped::ConstSharedPtr   &p)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    latest_wrench_ = *w;
    latest_pose_   = *p;

    // 1) reference‐collection mode?
    if (collecting_reference_ && init_count_ < 10) {
      fs_.emplace_back(w->wrench.force.x,  w->wrench.force.y,  w->wrench.force.z);
      ts_.emplace_back(w->wrench.torque.x, w->wrench.torque.y, w->wrench.torque.z);
      ps_.emplace_back(p->pose.position.x, p->pose.position.y,  p->pose.position.z);
      if (++init_count_ == 10) {
        zero_f_ = average(fs_);
        zero_t_ = average(ts_);
        zero_p_ = average(ps_);
        collecting_reference_ = false;
        openFile();
        recording_ = true;
        RCLCPP_INFO(get_logger(),
          "Collected 10 samples; reference set and logging started.");
      }
      return;  // skip logging until reference done
    }

    // 2) normal logging
    if (recording_) {
      // deltas
      Eigen::Vector3d f(latest_wrench_.wrench.force.x,
                        latest_wrench_.wrench.force.y,
                        latest_wrench_.wrench.force.z);
      Eigen::Vector3d t(latest_wrench_.wrench.torque.x,
                        latest_wrench_.wrench.torque.y,
                        latest_wrench_.wrench.torque.z);
      Eigen::Vector3d df = f - zero_f_, dt = t - zero_t_;

      Eigen::Vector3d pos(latest_pose_.pose.position.x,
                          latest_pose_.pose.position.y,
                          latest_pose_.pose.position.z);
      Eigen::Vector3d dp = pos - zero_p_;

      // forward tangent
      Eigen::Quaterniond q(
        latest_pose_.pose.orientation.w,
        latest_pose_.pose.orientation.x,
        latest_pose_.pose.orientation.y,
        latest_pose_.pose.orientation.z);
      Eigen::Vector3d forward = q * Eigen::Vector3d(0,0,1);

      auto now = get_clock()->now().seconds();
      csv_ << std::fixed << std::setprecision(6)
           << now << ","
           << df.x() << "," << df.y() << "," << df.z() << ","
           << dt.x() << "," << dt.y() << "," << dt.z() << ","
           << dp.x() << "," << dp.y() << "," << dp.z() << ","
           << forward.x() << "," << forward.y() << "," << forward.z()
           << "\n";
    }
  }

  Eigen::Vector3d average(const std::vector<Eigen::Vector3d> &v) {
    Eigen::Vector3d sum(0,0,0);
    for (auto &x : v) sum += x;
    return sum / double(v.size());
  }

  void openFile() {
    if (csv_.is_open()) csv_.close();
    std::ostringstream ss;
    ss << "sync_log_" << file_index_++ << ".csv";
    csv_.open(ss.str(), std::ios::out | std::ios::trunc);
    csv_ << "time,fx,fy,fz,tx,ty,tz,x,y,z,tangent_x,tangent_y,tangent_z\n";
  }

  // non‐blocking getchar() + reference trigger
  void keyboardLoop() {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON|ECHO);
    while (rclcpp::ok()) {
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
      int c = getchar();
      tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
      if (c=='r') {
        std::lock_guard<std::mutex> lk(mtx_);
        collecting_reference_ = true;
        init_count_ = 0;
        fs_.clear(); ts_.clear(); ps_.clear();
        recording_ = false;
        RCLCPP_INFO(get_logger(),
          "‘r’ pressed: collecting 10 reference samples...");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SyncLoggerNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
