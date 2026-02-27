#include <fstream>
#include <vector>
#include <chrono>
#include <string>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

enum State {
    WAITING_FOR_NDI_SIGNAL, // Mandatorily collect 10 samples to verify signal
    COLLECT_BASELINE,       // Phase to record initial baseline for the log
    MOVE_TO_SURFACE,        // Navigate to grid point
    PAUSE_SURFACE,          // 2s pause at surface
    MOVE_TO_PUSH,           // Execute push
    PAUSE_PUSH,             // 2s pause at depth
    RETRACT,                // Lift slightly above point
    ADVANCE_POINT,          // Move to next or finish
    DONE
};

struct Point3D { double x, y, z; };

class KukaPalpator : public rclcpp::Node {
public:
    KukaPalpator() : Node("kuka_palpator"), state_(WAITING_FOR_NDI_SIGNAL), point_index_(0), recording_(false) {

        // --- CONFIG FLAG ---
        // Set to TRUE for (Current - Baseline), FALSE for Raw Absolute coordinates
        use_relative_ndi_ = true; 

        kuka_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ves/kuka/servo_cp", 10);
        ves_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/ves/left/joint/servo_cp", 10);

        ndi_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/sensor_pose_raw", 10, std::bind(&KukaPalpator::ndi_callback, this, std::placeholders::_1));

        kuka_measured_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/ves/kuka/measured_cp", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                last_measured_kuka_ = msg->pose;
            });

        logfile_.open("kuka_palpation_experiment.csv");
        // Column headers
        logfile_ << "time,point_index,phase,cmd_px,cmd_py,cmd_pz,ndi_px,ndi_py,ndi_pz\n";

        // Define your palpation grid points (X, Y, Z relative to start)
        grid_points_ = {
            {-0.008, 0.000, 0.008}, {-0.005, -0.001, 0.008}, {-0.002, -0.006, 0.008},
            {-0.008, 0.000, 0.012}, {-0.005, -0.002, 0.012}, {-0.001, -0.004, 0.012}
        };

        push_depth_ = 0.006;
        retract_lift_ = 0.005; 
        push_direction_ = {0.0, 0.0, 1.0}; // Direction vector for push action

        timer_ = this->create_wall_timer(2000ms, std::bind(&KukaPalpator::state_machine, this));
        
        RCLCPP_INFO(this->get_logger(), "Node initialized. Mode: %s. Waiting for 10 NDI samples...", 
                    use_relative_ndi_ ? "RELATIVE" : "RAW");
    }

private:
    geometry_msgs::msg::PoseStamped get_target_msg(double dx, double dy, double dz) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "ves/kuka/base";
        msg.pose.position.x = kuka_home_pos_.x + dx;
        msg.pose.position.y = kuka_home_pos_.y + dy;
        msg.pose.position.z = kuka_home_pos_.z + dz;
        msg.pose.orientation = kuka_home_ori_;
        last_cmd_pose_ = msg.pose;
        return msg;
    }

    void ndi_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // ALWAYS collect 10 samples first to verify NDI is alive and calculate baseline
        if (state_ == WAITING_FOR_NDI_SIGNAL) {
            ref_samples_.push_back(msg->pose.position);
            if (ref_samples_.size() >= 10) {
                double sx=0, sy=0, sz=0;
                for(auto& p : ref_samples_) { sx+=p.x; sy+=p.y; sz+=p.z; }
                ndi_ref_ = {sx/10.0, sy/10.0, sz/10.0};
                
                // Lock current KUKA position as the origin for the relative grid
                kuka_home_pos_ = last_measured_kuka_.position;
                kuka_home_ori_ = last_measured_kuka_.orientation;
                
                RCLCPP_INFO(this->get_logger(), "✅ NDI Signal Verified. Baseline Calculated. Starting Baseline Phase.");
                state_ = COLLECT_BASELINE;
            }
            return;
        }

        if (!recording_) return;

        double t = this->get_clock()->now().nanoseconds() * 1e-9;
        
        // Logical Switch for Logging: uses the baseline calculated during the 10-sample wait
        double log_x = use_relative_ndi_ ? (msg->pose.position.x - ndi_ref_.x) : msg->pose.position.x;
        double log_y = use_relative_ndi_ ? (msg->pose.position.y - ndi_ref_.y) : msg->pose.position.y;
        double log_z = use_relative_ndi_ ? (msg->pose.position.z - ndi_ref_.z) : msg->pose.position.z;

        logfile_ << std::fixed << std::setprecision(6) << t << "," << point_index_ << "," << current_label_ << ","
                 << last_cmd_pose_.position.x << "," << last_cmd_pose_.position.y << "," << last_cmd_pose_.position.z << ","
                 << log_x << "," << log_y << "," << log_z << "\n";
    }

    void state_machine() {
        switch (state_) {
            case COLLECT_BASELINE:
                start_record("baseline", true);
                state_ = MOVE_TO_SURFACE;
                break;

            case MOVE_TO_SURFACE:
                recording_ = false;
                kuka_pub_->publish(get_target_msg(grid_points_[point_index_].x, grid_points_[point_index_].y, grid_points_[point_index_].z));
                state_ = PAUSE_SURFACE;
                break;

            case PAUSE_SURFACE:
                start_record("surface", true);
                state_ = MOVE_TO_PUSH;
                break;

            case MOVE_TO_PUSH: {
                recording_ = false;
                double px = grid_points_[point_index_].x + (push_depth_ * push_direction_.x);
                double py = grid_points_[point_index_].y + (push_depth_ * push_direction_.y);
                double pz = grid_points_[point_index_].z + (push_depth_ * push_direction_.z);
                kuka_pub_->publish(get_target_msg(px, py, pz));
                state_ = PAUSE_PUSH;
                break;
            }

            case PAUSE_PUSH:
                start_record("push", true);
                state_ = RETRACT;
                break;

            case RETRACT:
                recording_ = false;
                // Lift safely above surface (e.g. 5mm lift)
                kuka_pub_->publish(get_target_msg(grid_points_[point_index_].x, grid_points_[point_index_].y, grid_points_[point_index_].z - retract_lift_));
                state_ = ADVANCE_POINT;
                break;

            case ADVANCE_POINT:
                point_index_++;
                state_ = (point_index_ >= grid_points_.size()) ? DONE : MOVE_TO_SURFACE;
                break;

            case DONE:
                RCLCPP_INFO(this->get_logger(), "✅ All grid points completed successfully.");
                break;

            default:
                break;
        }
    }

    void start_record(std::string label, bool rec) {
        current_label_ = label;
        recording_ = rec;
        RCLCPP_INFO(this->get_logger(), "Phase: %s | Current Point Index: %zu", label.c_str(), point_index_);
    }

    // ROS 2 Comms
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr kuka_pub_, ves_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndi_sub_, kuka_measured_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data handling
    std::ofstream logfile_;
    State state_;
    size_t point_index_;
    bool recording_, use_relative_ndi_;
    std::string current_label_;

    // Hardware and Sensor Buffers
    geometry_msgs::msg::Pose last_measured_kuka_, last_cmd_pose_;
    geometry_msgs::msg::Point kuka_home_pos_, ndi_ref_;
    geometry_msgs::msg::Quaternion kuka_home_ori_;
    std::vector<geometry_msgs::msg::Point> ref_samples_;
    
    // Grid Specs
    std::vector<Point3D> grid_points_;
    double push_depth_, retract_lift_;
    Point3D push_direction_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KukaPalpator>());
    rclcpp::shutdown();
    return 0;
}