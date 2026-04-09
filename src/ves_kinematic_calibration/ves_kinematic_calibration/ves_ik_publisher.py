import csv
import numpy as np
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

# Import your kinematics class
from ves_kinematic_calibration.Virtuoso_kinematics import VirtuosoKinematics

# --------------------------------------------------
# Utilities
# --------------------------------------------------

def smoothstep(n):
    """Generates a smooth S-curve profile for interpolation."""
    t = np.linspace(0.0, 1.0, n)
    return t**2 * (3.0 - 2.0 * t)


def interpolate(start_cp, end_cp, dt=2.0, rate=50):
    """Linear interpolation in Cartesian space with smoothstep timing."""
    num_steps = max(2, int(dt * rate))
    ts = smoothstep(num_steps)

    p0 = np.array(start_cp[:3], dtype=float)
    p1 = np.array(end_cp[:3], dtype=float)

    waypoints = []
    for t in ts:
        p = (1.0 - t) * p0 + t * p1
        # Maintain identity orientation (quaternion [0,0,0,1])
        waypoints.append([
            float(p[0]),
            float(p[1]),
            float(p[2]),
            0.0, 0.0, 0.0, 1.0
        ])
    return waypoints


def normalize(v):
    """Normalizes a vector safely."""
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-12 else v


# --------------------------------------------------
# Palpation Node
# --------------------------------------------------

class PalpationGrid(Node):

    def __init__(self):
        super().__init__('palpation_grid')

        # 1. Initialize Kinematics
        # kappa_ik: Model used to "de-calculate" the ideal grid (23.9)
        # kappa_fk: Model used to "re-project" the points (60.0)
        self.kin = VirtuosoKinematics(kappa_ik=22.3, kappa_fk=60.0, tool_len=0.015)

        # Publishers / Subscribers
        self.cp_pub = self.create_publisher(PoseStamped, '/ves/left/joint/servo_cp', 10)
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)

        # Internal State
        self.current_joints = [0.0] * 5
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ndi = None

        # Parameters
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 3.0
        self.home_pause_time = 2.0
        self.baseline_time = 2.0

        self.push_depth = 0.007
        self.push_direction = normalize([0.0, -1, 0.0])
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]

        # 2. Define Original Ideal Grid Points
        raw_grid_points = [

            # # [-0.010, -0.003, 0.033, 0, 0, 0, 1],
            # [-0.008, -0.003, 0.033, 0, 0, 0, 1],
            # [-0.006, -0.0035, 0.033, 0, 0, 0, 1],
            # [-0.004, -0.004, 0.033, 0, 0, 0, 1],
            # [-0.002, -0.0045, 0.033, 0, 0, 0, 1],
            # [-0.000, -0.005, 0.033, 0, 0, 0, 1],
            # [0.002, -0.0055, 0.033, 0, 0, 0, 1],
            # [0.004, -0.006, 0.033, 0, 0, 0, 1],
    

            # # [-0.012, -0.003, 0.035, 0, 0, 0, 1],
            # [-0.010, -0.003, 0.035, 0, 0, 0, 1],
            # [-0.008, -0.003, 0.035, 0, 0, 0, 1],
            # [-0.006, -0.0035, 0.035, 0, 0, 0, 1],
            # [-0.004, -0.004, 0.035, 0, 0, 0, 1],
            # [-0.002, -0.0045, 0.035, 0, 0, 0, 1],
            # [-0.000, -0.005, 0.035, 0, 0, 0, 1],
            # [0.002, -0.0055, 0.035, 0, 0, 0, 1],
            # [0.004, -0.006, 0.035, 0, 0, 0, 1],
            # [0.006, -0.0065, 0.035, 0, 0, 0, 1],


            [-0.012, -0.003, 0.037, 0, 0, 0, 1],
            [-0.010, -0.003, 0.037, 0, 0, 0, 1],
            [-0.008, -0.003, 0.037, 0, 0, 0, 1],
            [-0.006, -0.0035, 0.037, 0, 0, 0, 1],
            [-0.004, -0.004, 0.037, 0, 0, 0, 1],
            [-0.002, -0.0045, 0.037, 0, 0, 0, 1],
            [-0.000, -0.005, 0.037, 0, 0, 0, 1],
            [0.002, -0.0055, 0.037, 0, 0, 0, 1],
            [0.004, -0.006, 0.037, 0, 0, 0, 1],
            [0.006, -0.0065, 0.037, 0, 0, 0, 1],


            [-0.012, -0.003, 0.039, 0, 0, 0, 1],
            [-0.010, -0.003, 0.039, 0, 0, 0, 1],
            [-0.008, -0.003, 0.039, 0, 0, 0, 1],
            [-0.006, -0.0035, 0.039, 0, 0, 0, 1],
            [-0.004, -0.004, 0.039, 0, 0, 0, 1],
            [-0.002, -0.0045, 0.039, 0, 0, 0, 1],
            [-0.000, -0.005, 0.039, 0, 0, 0, 1],
            [0.002, -0.0055, 0.039, 0, 0, 0, 1],
            [0.004, -0.006, 0.039, 0, 0, 0, 1],
            [0.006, -0.0065, 0.039, 0, 0, 0, 1],

            [-0.012, -0.003, 0.041, 0, 0, 0, 1],
            [-0.010, -0.003, 0.041, 0, 0, 0, 1],
            [-0.008, -0.003, 0.041, 0, 0, 0, 1],
            [-0.006, -0.0035, 0.041, 0, 0, 0, 1],
            [-0.004, -0.004, 0.041, 0, 0, 0, 1],
            [-0.002, -0.0045, 0.041, 0, 0, 0, 1],
            [-0.000, -0.005, 0.041, 0, 0, 0, 1],
            [0.002, -0.0055, 0.041, 0, 0, 0, 1],
            [0.004, -0.006, 0.041, 0, 0, 0, 1],
            [0.006, -0.0065, 0.041, 0, 0, 0, 1],

            [-0.012, -0.003, 0.043, 0, 0, 0, 1],
            [-0.010, -0.003, 0.043, 0, 0, 0, 1],
            [-0.008, -0.003, 0.043, 0, 0, 0, 1],
            [-0.006, -0.0035, 0.043, 0, 0, 0, 1],
            [-0.004, -0.004, 0.043, 0, 0, 0, 1],
            [-0.002, -0.0045, 0.043, 0, 0, 0, 1],
            [-0.000, -0.005, 0.043, 0, 0, 0, 1],
            [0.002, -0.0055, 0.043, 0, 0, 0, 1],
            [0.004, -0.006, 0.043, 0, 0, 0, 1],
            [0.006, -0.0065, 0.043, 0, 0, 0, 1],


            # [-0.012, -0.003, 0.045, 0, 0, 0, 1],
            # [-0.010, -0.003, 0.045, 0, 0, 0, 1],
            # [-0.008, -0.003, 0.045, 0, 0, 0, 1],
            # [-0.006, -0.0035, 0.045, 0, 0, 0, 1],
            # [-0.004, -0.004, 0.045, 0, 0, 0, 1],
            # [-0.002, -0.0045, 0.045, 0, 0, 0, 1],
            # [-0.000, -0.005, 0.045, 0, 0, 0, 1],
            # [0.002, -0.0055, 0.045, 0, 0, 0, 1],
            # [0.004, -0.006, 0.045, 0, 0, 0, 1],
            # [0.006, -0.0065, 0.045, 0, 0, 0, 1],


            # [-0.012, -0.003, 0.047, 0, 0, 0, 1],
            # [-0.010, -0.003, 0.047, 0, 0, 0, 1],
            # [-0.008, -0.003, 0.047, 0, 0, 0, 1],
            # [-0.006, -0.0035, 0.047, 0, 0, 0, 1],
            # [-0.004, -0.004, 0.047, 0, 0, 0, 1],
            # [-0.002, -0.0045, 0.047, 0, 0, 0, 1],
            # [-0.000, -0.005, 0.047, 0, 0, 0, 1],
            # [0.002, -0.0055, 0.047, 0, 0, 0, 1],
            # [0.004, -0.006, 0.047, 0, 0, 0, 1],
            # [0.006, -0.0065, 0.047, 0, 0, 0, 1],

            # [-0.012, -0.003, 0.049, 0, 0, 0, 1],
            # [-0.010, -0.003, 0.049, 0, 0, 0, 1],
            # [-0.008, -0.003, 0.049, 0, 0, 0, 1],
            # [-0.006, -0.0035, 0.049, 0, 0, 0, 1],
            # [-0.004, -0.004, 0.049, 0, 0, 0, 1],
            # [-0.002, -0.0045, 0.049, 0, 0, 0, 1],
            # [-0.000, -0.005, 0.049, 0, 0, 0, 1],
            # [0.002, -0.0055, 0.049, 0, 0, 0, 1],
            # [0.004, -0.006, 0.049, 0, 0, 0, 1],
            # [0.006, -0.0065, 0.049, 0, 0, 0, 1],


        ]

        # 3. Pre-process the Grid (Transformation via IK/FK cycle)
        self.surface_points = []
        self.push_points = []
        self.get_logger().info("🔄 Transforming Surface & Push points via curvature 60 map...")

        for pt in raw_grid_points:
            try:
                # --- Map Surface Point ---
                q_surf = self.kin.solve_ik(pt[:3])
                p_surf_mapped, _ = self.kin.solve_fk(q_surf)
                self.surface_points.append([float(p_surf_mapped[0]), float(p_surf_mapped[1]), float(p_surf_mapped[2]), 0, 0, 0, 1])
                
                # --- Map Push Point ---
                p_ideal_push = np.array(pt[:3]) + (self.push_depth * self.push_direction)
                q_push = self.kin.solve_ik(p_ideal_push)
                p_push_mapped, _ = self.kin.solve_fk(q_push)
                self.push_points.append([float(p_push_mapped[0]), float(p_push_mapped[1]), float(p_push_mapped[2]), 0, 0, 0, 1])
                
            except Exception as e:
                self.get_logger().error(f"Transformation failed for point {pt[:3]}: {e}")

        # State machine control
        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"
        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None
        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # CSV Logging Setup
        self.logfile = open('ik_final_grid_air.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time', 'point_index', 'phase',
            'cmd_px','cmd_py','cmd_pz','cmd_ox','cmd_oy','cmd_oz','cmd_ow',
            'ndi_px_rel','ndi_py_rel','ndi_pz_rel','ndi_ox','ndi_oy','ndi_oz','ndi_ow',
            'joint_0','joint_1','joint_2','joint_3','joint_4'
        ])

        # Start publishing loop
        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

    # --------------------------------------------------
    # Callbacks
    # --------------------------------------------------

    def joint_callback(self, msg):
        joints = list(msg.position) if msg.position else []
        joints += [0.0] * (5 - len(joints))
        self.current_joints = joints[:5]

    def ndi_callback(self, msg):
        pos = msg.pose.position
        ori = msg.pose.orientation

        if not self.reference_collected:
            self.reference_samples.append([pos.x, pos.y, pos.z])
            if len(self.reference_samples) >= 10:
                samples = np.array(self.reference_samples)
                self.reference_ndi = np.mean(samples, axis=0)
                self.reference_collected = True
                self.get_logger().info(f"✅ Ref Collected: {self.reference_ndi}")
                self.state = "GO_HOME_FOR_BASELINE"
                self.execute_state()
            return

        if not self.recording:
            return

        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            t, self.point_index, self.current_label,
            *self.last_command,
            pos.x - self.reference_ndi[0],
            pos.y - self.reference_ndi[1],
            pos.z - self.reference_ndi[2],
            ori.x, ori.y, ori.z, ori.w,
            *self.current_joints
        ])

    # --------------------------------------------------
    # State Logic
    # --------------------------------------------------

    def start_motion(self, start_cp, end_cp, next_state):
        self.current_trajectory = interpolate(start_cp, end_cp, dt=self.transition_time, rate=int(self.publish_rate))
        self.next_state_after_motion = next_state

    def start_pause(self, label, duration, next_state, record):
        if self.pause_timer is not None:
            return
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
        if self.state == "GO_HOME_FOR_BASELINE":
            self.start_motion(self.last_command, self.home_pose, "BASELINE_PAUSE")

        elif self.state == "BASELINE_PAUSE":
            self.get_logger().info("📊 Recording baseline at home")
            self.start_pause("baseline", self.baseline_time, "MOVE_TO_SURFACE", record=True)

        elif self.state == "MOVE_TO_SURFACE":
            target = self.surface_points[self.point_index]
            self.start_motion(self.last_command, target, "PAUSE_SURFACE")

        elif self.state == "PAUSE_SURFACE":
            self.get_logger().info(f"📍 Surface point {self.point_index}")
            self.start_pause("surface", self.pause_time, "MOVE_TO_PUSH", record=True)

        elif self.state == "MOVE_TO_PUSH":
            target = self.push_points[self.point_index]
            self.start_motion(self.last_command, target, "PAUSE_PUSH")

        elif self.state == "PAUSE_PUSH":
            self.get_logger().info(f"⬇️ Push point {self.point_index}")
            # FIXED: Added missing 'record' argument (True)
            self.start_pause("push", self.pause_time, "RETRACT", record=True)

        elif self.state == "RETRACT":
            target = self.surface_points[self.point_index]
            self.start_motion(self.last_command, target, "GO_HOME_AFTER_POINT")

        elif self.state == "GO_HOME_AFTER_POINT":
            self.start_motion(self.last_command, self.home_pose, "PAUSE_HOME")

        elif self.state == "PAUSE_HOME":
            self.start_pause("home_settle", self.home_pause_time, "ADVANCE_POINT", record=False)

        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.surface_points):
                self.state = "DONE"
                self.get_logger().info("✅ Palpation Grid Complete.")
            else:
                self.state = "MOVE_TO_SURFACE"
                self.execute_state()

    def publish_next_waypoint(self):
        if not self.reference_collected or self.pause_timer is not None:
            return

        if not self.current_trajectory:
            if self.next_state_after_motion is not None:
                self.state = self.next_state_after_motion
                self.next_state_after_motion = None
                self.execute_state()
            return

        target = self.current_trajectory.pop(0)
        px, py, pz, ox, oy, oz, ow = target

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = px, py, pz
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = ox, oy, oz, ow

        self.cp_pub.publish(msg)
        self.last_command = target

# --------------------------------------------------
# Main
# --------------------------------------------------

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