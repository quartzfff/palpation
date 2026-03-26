import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
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
# Calibration Collection Node
# --------------------------------------------------

class CalibrationCollector(Node):

    def __init__(self):
        super().__init__('calibration_collector')

        # Publishers / Subscribers
        self.cp_pub = self.create_publisher(PoseStamped, '/ves/left/joint/servo_cp', 10)
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)

        self.current_joints = [0.0] * 5

        # Parameters
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 2.0       # Recording duration at waypoint
        self.initial_wait = 2.0     # Recording duration at start (initial home)
        self.home_settle_wait = 1.0 # Non-recording wait at home between points

        # Zeroing / Reference
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ndi = None
        self.get_logger().info("⌛ Collecting 10 NDI samples to set zero reference...")

        # Poses
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]
        self.grid_points = [
            [-0.00900, -0.006, 0.015, 0, 0, 0, 1],
            [-0.00825, -0.006, 0.015, 0, 0, 0, 1],
            [-0.00750, -0.006, 0.015, 0, 0, 0, 1],
        ]

        # State machine
        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"
        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None
        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # CSV Logging
        self.logfile = open('calibration_data_raw.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time', 'point_index', 'phase',
            'cmd_px', 'cmd_py', 'cmd_pz',
            'ndi_px_raw', 'ndi_py_raw', 'ndi_pz_raw',
            'ndi_px_rel', 'ndi_py_rel', 'ndi_pz_rel',
            'ndi_ox', 'ndi_oy', 'ndi_oz', 'ndi_ow',
            'joint_0','joint_1','joint_2','joint_3','joint_4'
        ])

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

    def joint_callback(self, msg):
        joints = list(msg.position) if msg.position else []
        joints += [0.0] * (5 - len(joints))
        self.current_joints = joints[:5]

    def ndi_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation

        if not self.reference_collected:
            self.reference_samples.append(pos)
            if len(self.reference_samples) >= 10:
                self.reference_ndi = self.average_position(self.reference_samples)
                self.reference_collected = True
                self.get_logger().info(f"✅ Zeroed. Recording {self.initial_wait}s initial home baseline...")
                self.state = "INITIAL_PAUSE"
                self.execute_state()
            return

        if not self.recording:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            t, self.point_index, self.current_label,
            *self.last_command[:3],
            pos.x, pos.y, pos.z,
            pos.x - self.reference_ndi[0],
            pos.y - self.reference_ndi[1],
            pos.z - self.reference_ndi[2],
            ori.x, ori.y, ori.z, ori.w,
            *self.current_joints
        ])

    def start_motion(self, start_cp, end_cp, next_state):
        self.current_trajectory = interpolate(start_cp, end_cp, dt=self.transition_time, rate=int(self.publish_rate))
        self.next_state_after_motion = next_state

    def start_pause(self, label, duration, next_state, record):
        if self.pause_timer is not None: return
        self.current_label = label
        self.recording = record
        self.pause_timer = self.create_timer(duration, lambda: self.end_pause(next_state))

    def end_pause(self, next_state):
        self.recording = False
        if self.pause_timer:
            self.pause_timer.cancel()
            self.pause_timer = None
        self.state = next_state
        self.execute_state()

    def execute_state(self):
        # 1. INITIAL HOME: Record for 2 seconds
        if self.state == "INITIAL_PAUSE":
            self.start_pause("initial_home", self.initial_wait, "MOVE_TO_WAYPOINT", record=True)

        # 2. MOVE: to target
        elif self.state == "MOVE_TO_WAYPOINT":
            target = self.grid_points[self.point_index]
            self.get_logger().info(f"🚀 [Point {self.point_index}] Moving to Waypoint")
            self.start_motion(self.last_command, target, "RECORD_WAYPOINT")

        # 3. RECORD: for 2 seconds at waypoint
        elif self.state == "RECORD_WAYPOINT":
            self.get_logger().info(f"📸 [Point {self.point_index}] Recording waypoint...")
            self.start_pause("waypoint", self.pause_time, "MOVE_TO_HOME_RESET", record=True)

        # 4. HOME: Return home
        elif self.state == "MOVE_TO_HOME_RESET":
            self.get_logger().info(f"🏠 [Point {self.point_index}] Returning Home")
            self.start_motion(self.last_command, self.home_pose, "HOME_SETTLE_PAUSE")

        # 5. SETTLE: Wait 1s at home, NO RECORDING
        elif self.state == "HOME_SETTLE_PAUSE":
            self.get_logger().info(f"⌛ Settling at home for {self.home_settle_wait}s (Not recording)")
            self.start_pause("home_settle", self.home_settle_wait, "ADVANCE_POINT", record=False)

        # 6. CYCLE
        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.grid_points):
                self.state = "DONE"
            else:
                self.state = "MOVE_TO_WAYPOINT"
            self.execute_state()

        elif self.state == "DONE":
            self.get_logger().info("✅ Calibration run complete.")

    def publish_next_waypoint(self):
        if not self.reference_collected or self.pause_timer:
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
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = ox, oy, oz, ow

        self.cp_pub.publish(msg)
        self.last_command = [px, py, pz, ox, oy, oz, ow]

    @staticmethod
    def average_position(positions):
        n = len(positions)
        return (sum(p.x for p in positions)/n, sum(p.y for p in positions)/n, sum(p.z for p in positions)/n)

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