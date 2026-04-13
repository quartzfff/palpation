import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState


# --------------------------------------------------
# Utilities
# --------------------------------------------------

def smoothstep(n):
    t = np.linspace(0.0, 1.0, n)
    return t**2 * (3.0 - 2.0 * t)

def interpolate(start_cp, end_cp, dt=2.0, rate=50):
    num_steps = max(2, int(dt * rate))
    ts = smoothstep(num_steps)

    p0 = np.array(start_cp[:3], dtype=float)
    p1 = np.array(end_cp[:3], dtype=float)

    waypoints = []
    for t in ts:
        p = (1.0 - t) * p0 + t * p1
        waypoints.append([
            float(p[0]), float(p[1]), float(p[2]),
            0.0, 0.0, 0.0, 1.0
        ])
    return waypoints


# --------------------------------------------------
# Node
# --------------------------------------------------

class CalibrationCollector(Node):

    def __init__(self):
        super().__init__('calibration_collector')

        # -----------------------
        # Publishers / Subscribers
        # -----------------------
        self.cp_pub = self.create_publisher(
            PoseStamped, '/ves/left/joint/servo_cp', 10)

        self.ndi_sub = self.create_subscription(
            PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)

        self.joint_sub = self.create_subscription(
            JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)

        self.force_sub = self.create_subscription(
            WrenchStamped, '/netft_data', self.force_callback, 10)

        self.current_joints = [0.0] * 5
        self.current_force = [0.0, 0.0, 0.0]

        # -----------------------
        # Parameters
        # -----------------------
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 2.0
        self.home_settle_wait = 2.0   # 👈 used now

        # -----------------------
        # Poses
        # -----------------------
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]

        self.grid_points = [
            [0.016, 0.000, 0.030, 0, 0, 0, 1],
            [0.016, 0.009, 0.030, 0, 0, 0, 1],



            [0.016, -0.00, 0.030, 0, 0, 0, 1],
            [0.016, -0.011, 0.030, 0, 0, 0, 1],


            [0.016, -0.00, 0.030, 0, 0, 0, 1],
             [0.007, -0.0, 0.032, 0, 0, 0, 1],

            [0.016, -0.00, 0.030, 0, 0, 0, 1],
            [0.025, -0.00, 0.032, 0, 0, 0, 1],



            [-0.016,0.0,0.03,0,0,0,1],
            [-0.016,0.009,0.03,0,0,0,1],

            [-0.016,0.0,0.03,0,0,0,1],
            [-0.016,-0.010,0.03,0,0,0,1],

            [-0.016,0.0,0.03,0,0,0,1],
            [-0.007,0.0,0.033,0,0,0,1],

            [-0.016,0.0,0.03,0,0,0,1],
            [-0.025,0.0,0.031,0,0,0,1],


            [0,0.016,0.03,0,0,0,1],
            [-0.0,0.025,0.032,0,0,0,1],

            [0.0,0.016,0.03,0,0,0,1],
            [-0.0,0.007,0.033,0,0,0,1],

            [-0.0,0.016,0.03,0,0,0,1],
            [0.011,0.016,0.033,0,0,0,1],

            [-0.0,0.016,0.03,0,0,0,1],
            [-0.009,0.016,0.031,0,0,0,1],


            [0,-0.016,0.03,0,0,0,1],
            [-0.0,-0.006,0.033,0,0,0,1],

            [0.0,-0.016,0.03,0,0,0,1],
            [-0.0,-0.025,0.033,0,0,0,1],

            [-0.0,-0.016,0.03,0,0,0,1],
            [0.010,-0.016,0.033,0,0,0,1],

            [-0.0,-0.016,0.03,0,0,0,1],
            [-0.009,-0.016,0.031,0,0,0,1],





        ]

        # -----------------------
        # State machine
        # -----------------------
        self.point_index = 0
        self.state = "MOVE_TO_FIRST"
        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None

        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # -----------------------
        # Logging
        # -----------------------
        self.logfile = open('force_15mm_full_002.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)

        self.csv_writer.writerow([
            'time', 'pair_id', 'phase',
            'cmd_px', 'cmd_py', 'cmd_pz',
            'ndi_px', 'ndi_py', 'ndi_pz',
            'ndi_ox', 'ndi_oy', 'ndi_oz', 'ndi_ow',
            'fx','fy','fz',
            'joint_0','joint_1','joint_2','joint_3','joint_4'
        ])

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_next_waypoint
        )

        self.execute_state()

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def joint_callback(self, msg):
        joints = list(msg.position) if msg.position else []
        joints += [0.0] * (5 - len(joints))
        self.current_joints = joints[:5]

    def force_callback(self, msg):
        f = msg.wrench.force
        self.current_force = [f.x, f.y, f.z]

    def ndi_callback(self, msg):
        if not self.recording:
            return

        pos = msg.pose.position
        ori = msg.pose.orientation

        t = self.get_clock().now().nanoseconds * 1e-9
        fx, fy, fz = self.current_force

        self.csv_writer.writerow([
            t,
            self.point_index // 2,
            self.current_label,
            *self.last_command[:3],
            pos.x, pos.y, pos.z,
            ori.x, ori.y, ori.z, ori.w,
            fx, fy, fz,
            *self.current_joints
        ])

    # --------------------------------------------------
    # Motion helpers
    # --------------------------------------------------

    def start_motion(self, target, next_state):
        self.current_trajectory = interpolate(
            self.last_command,
            target,
            dt=self.transition_time,
            rate=int(self.publish_rate)
        )
        self.next_state_after_motion = next_state

    def start_pause(self, label, next_state):
        if self.pause_timer is not None:
            return

        self.current_label = label
        self.recording = True

        self.pause_timer = self.create_timer(
            self.pause_time,
            lambda: self.end_pause(next_state)
        )

    def start_home_settle(self, next_state):
        if self.pause_timer is not None:
            return

        self.get_logger().info("⏳ Settling at home")
        self.recording = False  # 👈 important: do NOT log at home

        self.pause_timer = self.create_timer(
            self.home_settle_wait,
            lambda: self.end_pause(next_state)
        )

    def end_pause(self, next_state):
        self.recording = False

        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None

        self.state = next_state
        self.execute_state()

    # --------------------------------------------------
    # State machine
    # --------------------------------------------------

    def execute_state(self):

        if self.state == "MOVE_TO_FIRST":
            target = self.grid_points[self.point_index]
            self.get_logger().info(f"🚀 Move to WP1 (pair {self.point_index//2})")
            self.start_motion(target, "RECORD_FIRST")

        elif self.state == "RECORD_FIRST":
            self.get_logger().info("📸 Record WP1")
            self.start_pause("wp1", "MOVE_TO_SECOND")

        elif self.state == "MOVE_TO_SECOND":
            target = self.grid_points[self.point_index + 1]
            self.get_logger().info("🚀 Move to WP2")
            self.start_motion(target, "RECORD_SECOND")

        elif self.state == "RECORD_SECOND":
            self.get_logger().info("📸 Record WP2")
            self.start_pause("wp2", "GO_HOME")

        elif self.state == "GO_HOME":
            self.get_logger().info("🏠 Go Home")
            self.start_motion(self.home_pose, "WAIT_AT_HOME")

        elif self.state == "WAIT_AT_HOME":
            self.start_home_settle("NEXT_PAIR")

        elif self.state == "NEXT_PAIR":
            self.point_index += 2

            if self.point_index >= len(self.grid_points) - 1:
                self.state = "DONE"
            else:
                self.state = "MOVE_TO_FIRST"

            self.execute_state()

        elif self.state == "DONE":
            self.get_logger().info("✅ Calibration complete.")

    # --------------------------------------------------
    # Publisher
    # --------------------------------------------------

    def publish_next_waypoint(self):

        if self.pause_timer:
            return

        if not self.current_trajectory:
            if self.next_state_after_motion is not None:
                self.state = self.next_state_after_motion
                self.next_state_after_motion = None
                self.execute_state()
            return

        px, py, pz, ox, oy, oz, ow = self.current_trajectory.pop(0)

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        msg.pose.position.x = px
        msg.pose.position.y = py
        msg.pose.position.z = pz
        msg.pose.orientation.x = ox
        msg.pose.orientation.y = oy
        msg.pose.orientation.z = oz
        msg.pose.orientation.w = ow

        self.cp_pub.publish(msg)
        self.last_command = [px, py, pz, ox, oy, oz, ow]


# --------------------------------------------------
# Main
# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCollector()

    try:
        rclpy.spin(node)
    finally:
        node.logfile.flush()
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()