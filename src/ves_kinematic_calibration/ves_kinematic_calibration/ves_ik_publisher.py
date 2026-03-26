import csv
import numpy as np
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ves_kinematic_calibration.Virtuoso_kinematics import VirtuosoKinematics


# --- Utilities ---
def smoothstep(n):
    t = np.linspace(0.0, 1.0, n)
    return t ** 2 * (3.0 - 2.0 * t)


def interpolate(start_cp, end_cp, dt=2.0, rate=50):
    num_steps = max(2, int(dt * rate))
    ts = smoothstep(num_steps)
    p0 = np.array(start_cp[:3], dtype=float)
    p1 = np.array(end_cp[:3], dtype=float)
    waypoints = []
    for t in ts:
        p = (1.0 - t) * p0 + t * p1
        waypoints.append([float(p[0]), float(p[1]), float(p[2]), 0.0, 0.0, 0.0, 1.0])
    return waypoints


def normalize(v):
    norm = np.linalg.norm(v)
    return np.array(v) / norm if norm > 1e-12 else np.array(v)


# --- Palpation Node ---
class PalpationGrid(Node):
    def __init__(self):
        super().__init__('palpation_grid')

        # 1. Initialize Kinematics (Strictly METERS)
        self.kin = VirtuosoKinematics(kappa=28.5, tool_len=0.015)

        # Publishers / Subscribers
        self.jp_pub = self.create_publisher(JointState, '/ves/left/joint/servo_jp', 10)
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)

        # Parameters (METERS)
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.record_pause_time = 2.0
        self.home_pause_time = 2.0

        self.push_depth = 0.001  # 7mm
        self.push_direction = normalize([-0.1, -0.8, -0.1])
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]

        # Waypoint Grid (METERS)
        self.grid_points = [
            [0, -0.006, 0.025, 0, 0, 0, 1],
 
        ]

        # State Variables
        self.current_joints = [0.0] * 5
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ndi = None
        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"
        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None
        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # CSV Setup (Logging in Meters)
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = f'palpation_IKdata_{timestr}.csv'
        self.logfile = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time', 'pt_idx', 'phase',
            'cmd_px_m', 'cmd_py_m', 'cmd_pz_m',
            'ndi_raw_px', 'ndi_raw_py', 'ndi_raw_pz',
            'ndi_rel_px_m', 'ndi_rel_py_m', 'ndi_rel_pz_m',
            'ndi_ox', 'ndi_oy', 'ndi_oz', 'ndi_ow',
            'ir', 'or', 'it', 'ot', 'tool'
        ])

        # Pre-flight Check (Meters)
        self.get_logger().info("Checking grid feasibility (Meters)...")
        errors = self.kin.check_grid_feasibility(self.grid_points, self.push_depth, self.push_direction)
        if errors:
            for err in errors:
                self.get_logger().error(f"❌ {err}")
        else:
            self.get_logger().info("✅ All points reachable.")

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

    def joint_callback(self, msg):
        if msg.position:
            self.current_joints = list(msg.position) + [0.0] * (5 - len(msg.position))

    def ndi_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation

        if not self.reference_collected:
            self.reference_samples.append([pos.x, pos.y, pos.z])
            if len(self.reference_samples) >= 10:
                self.reference_ndi = np.mean(self.reference_samples, axis=0)
                self.reference_collected = True
                self.get_logger().info(f"✅ Ref Collected: {self.reference_ndi}")
                self.state = "GO_HOME_FOR_START"
                self.execute_state()
            return

        if self.recording:
            t = self.get_clock().now().nanoseconds * 1e-9
            q = self.get_current_commanded_joints()
            self.csv_writer.writerow([
                t, self.point_index, self.current_label,
                *self.last_command[:3],
                pos.x, pos.y, pos.z,
                (pos.x - self.reference_ndi[0]),
                (pos.y - self.reference_ndi[1]),
                (pos.z - self.reference_ndi[2]),
                ori.x, ori.y, ori.z, ori.w,
                *q
            ])

    def get_current_commanded_joints(self):
        try:
            return self.kin.solve_ik(self.last_command[:3])
        except:
            return [0.0] * 5

    def start_motion(self, start_cp, end_cp, next_state):
        self.current_trajectory = interpolate(start_cp, end_cp, dt=self.transition_time, rate=self.publish_rate)
        self.next_state_after_motion = next_state

    def start_pause(self, label, duration, next_state, record):
        self.current_label = label
        self.recording = record
        self.pause_timer = self.create_timer(duration, lambda: self.end_pause(next_state))

    def end_pause(self, next_state):
        self.recording = False
        if self.pause_timer:
            self.pause_timer.cancel()
        self.state = next_state
        self.execute_state()

    def execute_state(self):
        if self.state == "GO_HOME_FOR_START":
            self.start_motion(self.last_command, self.home_pose, "MOVE_TO_SURFACE")
        elif self.state == "MOVE_TO_SURFACE":
            self.start_motion(self.last_command, self.grid_points[self.point_index], "PAUSE_SURFACE")
        elif self.state == "PAUSE_SURFACE":
            self.start_pause("surface", self.record_pause_time, "MOVE_TO_PUSH", record=True)
        elif self.state == "MOVE_TO_PUSH":
            p_push = np.array(self.grid_points[self.point_index][:3]) + self.push_depth * self.push_direction
            push_pose = [float(p_push[0]), float(p_push[1]), float(p_push[2]), 0, 0, 0, 1]
            self.start_motion(self.last_command, push_pose, "PAUSE_PUSH")
        elif self.state == "PAUSE_PUSH":
            self.start_pause("push", self.record_pause_time, "GO_HOME_AFTER_POINT", record=True)
        elif self.state == "GO_HOME_AFTER_POINT":
            self.start_motion(self.last_command, self.home_pose, "PAUSE_HOME")
        elif self.state == "PAUSE_HOME":
            self.start_pause("home_wait", self.home_pause_time, "ADVANCE_POINT", record=False)
        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.grid_points):
                self.get_logger().info("✅ Palpation Grid Complete.")
            else:
                self.state = "MOVE_TO_SURFACE"
                self.execute_state()

    def publish_next_waypoint(self):
        if not self.reference_collected or self.pause_timer is not None:
            return
        if not self.current_trajectory:
            if self.next_state_after_motion:
                self.state, self.next_state_after_motion = self.next_state_after_motion, None
                self.execute_state()
            return

        target_pose = self.current_trajectory.pop(0)
        try:
            # Kinematics already returns Meters/Radians
            q = self.kin.solve_ik(target_pose[:3])

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ["inner_rotation", "outer_rotation", "inner_translation", "outer_translation", "tool"]
            msg.position = q  # q[2] and q[3] are in Meters
            self.jp_pub.publish(msg)
            self.last_command = target_pose
        except Exception as e:
            self.get_logger().error(f"IK Failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PalpationGrid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logfile.flush()
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()