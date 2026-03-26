import csv
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# --------------------------------------------------
# Utilities
# --------------------------------------------------

def smoothstep(n):
    t = np.linspace(0.0, 1.0, n)
    return t**2 * (3.0 - 2.0 * t)

def interpolate_joint_space(start_js, end_js, dt=2.0, rate=50):
    num_steps = max(2, int(dt * rate))
    ts = smoothstep(num_steps)
    
    start = np.array(start_js)
    end = np.array(end_js)
    
    waypoints = []
    for t in ts:
        p = (1.0 - t) * start + t * end
        waypoints.append(p.tolist())
    return waypoints

# --------------------------------------------------
# Joint Space Calibration Node
# --------------------------------------------------

class JointCalibrationCollector(Node):

    def __init__(self):
        super().__init__('joint_calibration_collector')

        # Joint Publisher (servo_jp for control)
        self.joint_pub = self.create_publisher(JointState, '/ves/left/joint/servo_jp', 10)
        
        # Subscribers
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)
        self.cp_sub = self.create_subscription(PoseStamped, '/ves/left/joint/setpoint_cp', self.cp_callback, 10)

        # State Variables
        self.current_measured_joints = [0.0] * 5
        self.current_cp = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0] # px, py, pz, ox, oy, oz, ow
        
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 2.0        # seconds recording at waypoint
        self.initial_wait = 2.0      # wait at base first
        self.home_settle_wait = 2.0  # wait at home between points

        # Zeroing (NDI Only)
        self.reference_collected = False
        self.reference_samples_ndi = []
        self.reference_ndi = None
        self.get_logger().info("⌛ Collecting 10 NDI samples for zeroing...")

        # Joint Waypoints [ir, or, it, ot, tool]
        self.home_joints = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_grid = [
            [0.0, 0.0, 0.01, 0.005, 0.0],
            [0.0, 0.0, 0.011, 0.006, 0.0],
            [0.0, 0.0, 0.012, 0.007, 0.0],
            [0.0, 0.0, 0.013, 0.008, 0.0],
            [0.0, 0.0, 0.014, 0.009, 0.0],
            [0.0, 0.0, 0.015, 0.010, 0.0],
            [0.0, 0.0, 0.016, 0.011, 0.0],
            [0.0, 0.0, 0.017, 0.012, 0.0],
            [0.0, 0.0, 0.018, 0.013, 0.0],
            [0.0, 0.0, 0.019, 0.014, 0.0],
            [0.0, 0.0, 0.020, 0.015, 0.0],
            [0.0, 0.0, 0.021, 0.016, 0.0],
            [0.0, 0.0, 0.022, 0.017, 0.0],
            [0.0, 0.0, 0.023, 0.018, 0.0],
            [0.0, 0.0, 0.024, 0.019, 0.0],


            [0.0, 3.14, 0.01, 0.005, 0.0],
            [0.0, 3.14, 0.011, 0.006, 0.0],
            [0.0, 3.14, 0.012, 0.007, 0.0],
            [0.0, 3.14, 0.013, 0.008, 0.0],
            [0.0, 3.14, 0.014, 0.009, 0.0],
            [0.0, 3.14, 0.015, 0.010, 0.0],
            [0.0, 0.0, 0.016, 0.011, 0.0],
            [0.0, 0.0, 0.017, 0.012, 0.0],
            [0.0, 0.0, 0.018, 0.013, 0.0],
            [0.0, 0.0, 0.019, 0.014, 0.0],
            [0.0, 0.0, 0.020, 0.015, 0.0],
            [0.0, 0.0, 0.021, 0.016, 0.0],
            [0.0, 0.0, 0.022, 0.017, 0.0],
            [0.0, 0.0, 0.023, 0.018, 0.0],
            [0.0, 0.0, 0.024, 0.019, 0.0],

            [0.0, 1.57, 0.01, 0.005, 0.0],
            [0.0, 1.57, 0.011, 0.006, 0.0],
            [0.0, 1.57, 0.012, 0.007, 0.0],
            [0.0, 1.57, 0.013, 0.008, 0.0],
            [0.0, 1.57, 0.014, 0.009, 0.0],
            [0.0, 1.57, 0.015, 0.010, 0.0],
            [0.0, 1.57, 0.016, 0.011, 0.0],
            [0.0, 1.57, 0.017, 0.012, 0.0],
            [0.0, 1.57, 0.018, 0.013, 0.0],
            [0.0, 1.57, 0.019, 0.014, 0.0],
            [0.0, 1.57, 0.020, 0.015, 0.0],
            [0.0, 1.57, 0.021, 0.016, 0.0],
            [0.0, 1.57, 0.022, 0.017, 0.0],
            [0.0, 1.57, 0.023, 0.018, 0.0],
            [0.0, 1.57, 0.024, 0.019, 0.0],

            [0.0, -1.57, 0.01, 0.005, 0.0],
            [0.0, -1.57, 0.011, 0.006, 0.0],
            [0.0, -1.57, 0.012, 0.007, 0.0],
            [0.0, -1.57, 0.013, 0.008, 0.0],
            [0.0, -1.57, 0.014, 0.009, 0.0],
            [0.0, -1.57, 0.015, 0.010, 0.0],
            [0.0, -1.57, 0.016, 0.011, 0.0],
            [0.0, -1.57, 0.017, 0.012, 0.0],
            [0.0, -1.57, 0.018, 0.013, 0.0],
            [0.0, -1.57, 0.019, 0.014, 0.0],
            [0.0, -1.57, 0.020, 0.015, 0.0],
            [0.0, -1.57, 0.021, 0.016, 0.0],
            [0.0, -1.57, 0.022, 0.017, 0.0],
            [0.0, -1.57, 0.023, 0.018, 0.0],
            [0.0, -1.57, 0.024, 0.019, 0.0],

        ]

        # Logic Control
        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"
        self.current_trajectory = []
        self.last_joint_cmd = self.home_joints.copy()
        self.next_state_after_motion = None
        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # CSV Setup
        self.logfile = open('joint_calibration_jp_002.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time', 'point_index', 'phase',
            'cmd_j0','cmd_j1','cmd_j2','cmd_j3','cmd_j4',
            'ndi_px_raw','ndi_py_raw','ndi_pz_raw',
            'ndi_px_rel','ndi_py_rel','ndi_pz_rel',
            'ndi_ox','ndi_oy','ndi_oz','ndi_ow',  
            'cp_px','cp_py','cp_pz',               # Measured CP (Robot Frame)
            'msr_j0','msr_j1','msr_j2','msr_j3','msr_j4'
        ])

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

    def joint_callback(self, msg):
        if msg.position:
            self.current_measured_joints = list(msg.position)[:5]

    def cp_callback(self, msg):
        p = msg.pose.position
        o = msg.pose.orientation
        self.current_cp = [p.x, p.y, p.z, o.x, o.y, o.z, o.w]

    def ndi_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation # Capture orientation

        # Handle NDI Reference Collection
        if not self.reference_collected:
            self.reference_samples_ndi.append(pos)
            if len(self.reference_samples_ndi) >= 10:
                self.reference_ndi = self.average_position(self.reference_samples_ndi)
                self.reference_collected = True
                self.get_logger().info(f"✅ NDI Zeroed. Waiting {self.initial_wait}s at base...")
                self.state = "INITIAL_PAUSE"
                self.execute_state()
            return

        if not self.recording:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            t, self.point_index, self.current_label,
            *self.last_joint_cmd,
            pos.x, pos.y, pos.z,                          # NDI Raw
            pos.x - self.reference_ndi[0],                # NDI Rel
            pos.y - self.reference_ndi[1],
            pos.z - self.reference_ndi[2],
            ori.x, ori.y, ori.z, ori.w, 
            self.current_cp[0], self.current_cp[1], self.current_cp[2], # Robot CP 
            *self.current_measured_joints
        ])

    def start_motion(self, start, end, next_state):
        self.current_trajectory = interpolate_joint_space(start, end, dt=self.transition_time, rate=self.publish_rate)
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
        # 1. Wait at base for 2 seconds (No recording)
        if self.state == "INITIAL_PAUSE":
            self.start_pause("initial_wait", self.initial_wait, "MOVE_TO_WAYPOINT", record=True)

        # 2. Move to Waypoint
        elif self.state == "MOVE_TO_WAYPOINT":
            target = self.joint_grid[self.point_index]
            self.get_logger().info(f"🚀 [Joint Point {self.point_index}] Moving to Waypoint")
            self.start_motion(self.last_joint_cmd, target, "RECORD_WAYPOINT")

        # 3. Record at Waypoint
        elif self.state == "RECORD_WAYPOINT":
            self.get_logger().info(f"📸 Recording Point {self.point_index}...")
            self.start_pause("waypoint", self.pause_time, "MOVE_TO_HOME_RESET", record=True)

        # 4. Return Home
        elif self.state == "MOVE_TO_HOME_RESET":
            self.get_logger().info(f"🏠 [Point {self.point_index}] Resetting to Home...")
            self.start_motion(self.last_joint_cmd, self.home_joints, "HOME_SETTLE_PAUSE")

        # 5. Wait at Home for 1 second (No recording)
        elif self.state == "HOME_SETTLE_PAUSE":
            self.get_logger().info(f"⌛ Settling at home for {self.home_settle_wait}s...")
            self.start_pause("home_settle", self.home_settle_wait, "ADVANCE_POINT", record=False)

        # 6. Advance
        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.joint_grid):
                self.state = "DONE"
            else:
                self.state = "MOVE_TO_WAYPOINT"
            self.execute_state()

        elif self.state == "DONE":
            self.get_logger().info("✅ Joint Calibration run complete.")

    def publish_next_waypoint(self):
        if not self.reference_collected or self.pause_timer:
            return

        if not self.current_trajectory:
            if self.next_state_after_motion:
                self.state = self.next_state_after_motion
                self.next_state_after_motion = None
                self.execute_state()
            return

        j_cmd = self.current_trajectory.pop(0)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['ir', 'or', 'it', 'ot', 'tool']
        msg.position = j_cmd
        self.joint_pub.publish(msg)
        self.last_joint_cmd = j_cmd

    @staticmethod
    def average_position(positions):
        n = len(positions)
        return (sum(p.x for p in positions)/n, sum(p.y for p in positions)/n, sum(p.z for p in positions)/n)

def main(args=None):
    rclpy.init(args=args)
    node = JointCalibrationCollector()
    try:
        rclpy.spin(node)
    finally:
        node.logfile.flush()
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()