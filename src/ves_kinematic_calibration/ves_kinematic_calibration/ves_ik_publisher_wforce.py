import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState

# Import your kinematics class
from ves_kinematic_calibration.Virtuoso_kinematics import VirtuosoKinematics

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
# Palpation Node
# --------------------------------------------------

class PalpationGrid(Node):

    def __init__(self):
        super().__init__('palpation_grid')

        # 1. Initialize Kinematics
        self.kin = VirtuosoKinematics(kappa_ik=60, kappa_fk=60.0, tool_len=0.02)

        # Publishers / Subscribers
        self.cp_pub = self.create_publisher(PoseStamped, '/ves/left/joint/servo_cp', 10)
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/ves/left/joint/setpoint_jp', self.joint_callback, 10)
        self.force_sub = self.create_subscription(WrenchStamped, '/netft_data', self.force_callback, 10)

        # Internal State
        self.current_joints = [0.0] * 5
        self.current_force = [0.0, 0.0, 0.0]
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ndi = None

        # Parameters
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 3.0
        self.home_pause_time = 2.0
        self.baseline_time = 2.0
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]

        # 2. Define Explicit Pairs: [Surface Point, Push-to Point]
        # This allows you to define unique directions/depths for every site.
        raw_pairs = [
            # Site 1: [px, py, pz, ox, oy, oz, ow]
           #[ [0.012, -0.005, 0.025, 0, 0, 0, 1], [0.012, -0.012, 0.025, 0, 0, 0, 1] ],
            # Site 2: Push in a different direction (e.g., diagonal)
            [ [0.004, 0.0, 0.027, 0, 0, 0, 1],    [0.008, -0.005, 0.030, 0, 0, 0, 1] ],
        ]

        # 3. Pre-process via IK/FK
        self.surface_points = []
        self.push_points = []
        self.get_logger().info("🔄 Transforming explicit pairs via kinematics...")

        for surf_pt, push_pt in raw_pairs:
            try:
                # Map Surface Point
                q_surf = self.kin.solve_ik(surf_pt[:3])
                p_surf_mapped, _ = self.kin.solve_fk(q_surf)
                self.surface_points.append([float(p_surf_mapped[0]), float(p_surf_mapped[1]), float(p_surf_mapped[2]), 0, 0, 0, 1])
                
                # Map Push Point
                q_push = self.kin.solve_ik(push_pt[:3])
                p_push_mapped, _ = self.kin.solve_fk(q_push)
                self.push_points.append([float(p_push_mapped[0]), float(p_push_mapped[1]), float(p_push_mapped[2]), 0, 0, 0, 1])
            except Exception as e:
                self.get_logger().error(f"Transformation failed: {e}")

        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"
        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None
        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # Logging Setup
        self.logfile = open('multi_direction_palpation.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time', 'site_index', 'phase',
            'cmd_px','cmd_py','cmd_pz','cmd_ox','cmd_oy','cmd_oz','cmd_ow',
            'ndi_px_rel','ndi_py_rel','ndi_pz_rel','ndi_ox','ndi_oy','ndi_oz','ndi_ow',
            'fx', 'fy', 'fz',
            'joint_0','joint_1','joint_2','joint_3','joint_4'
        ])

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

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
        if not self.reference_collected:
            pos = msg.pose.position
            self.reference_samples.append([pos.x, pos.y, pos.z])
            if len(self.reference_samples) >= 10:
                self.reference_ndi = np.mean(self.reference_samples, axis=0)
                self.reference_collected = True
                self.state = "GO_HOME_FOR_BASELINE"
                self.execute_state()
            return

        if not self.recording:
            return

        pos = msg.pose.position
        ori = msg.pose.orientation
        t = self.get_clock().now().nanoseconds * 1e-9
        self.csv_writer.writerow([
            t, self.point_index, self.current_label,
            *self.last_command,
            pos.x - self.reference_ndi[0], pos.y - self.reference_ndi[1], pos.z - self.reference_ndi[2],
            ori.x, ori.y, ori.z, ori.w,
            *self.current_force,
            *self.current_joints
        ])

    # --------------------------------------------------
    # State Logic
    # --------------------------------------------------

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
        """
        1. Home Baseline (Record)
        2. Move to Site
        3. Record at Site
        4. Move to Push Point
        5. Record at Push Point
        6. Move back to Site (No record)
        7. Go Home
        """
        if self.state == "GO_HOME_FOR_BASELINE":
            self.start_motion(self.last_command, self.home_pose, "BASELINE_PAUSE")

        elif self.state == "BASELINE_PAUSE":
            self.get_logger().info(f"📊 Site {self.point_index}: Home Baseline")
            self.start_pause("baseline", self.baseline_time, "MOVE_TO_SITE", record=True)

        elif self.state == "MOVE_TO_SITE":
            self.start_motion(self.last_command, self.surface_points[self.point_index], "PAUSE_AT_SITE")

        elif self.state == "PAUSE_AT_SITE":
            self.get_logger().info(f"📍 Site {self.point_index}: Surface")
            self.start_pause("surface_pre_push", self.pause_time, "MOVE_TO_PUSH", record=True)

        elif self.state == "MOVE_TO_PUSH":
            self.start_motion(self.last_command, self.push_points[self.point_index], "PAUSE_AT_PUSH")

        elif self.state == "PAUSE_AT_PUSH":
            self.get_logger().info(f"⬇️ Site {self.point_index}: Push Depth reached")
            self.start_pause("push_active", self.pause_time, "RETRACT_TO_SITE", record=True)

        elif self.state == "RETRACT_TO_SITE":
            self.get_logger().info("⬆️ Retracting to site (No recording)")
            self.start_motion(self.last_command, self.surface_points[self.point_index], "GO_HOME_FINAL")

        elif self.state == "GO_HOME_FINAL":
            self.start_motion(self.last_command, self.home_pose, "PAUSE_AT_HOME")

        elif self.state == "PAUSE_AT_HOME":
            self.start_pause("home_settle", self.home_pause_time, "ADVANCE_POINT", record=False)

        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.surface_points):
                self.get_logger().info("✅ All sites complete.")
                self.state = "DONE"
            else:
                self.state = "GO_HOME_FOR_BASELINE"
                self.execute_state()

    def publish_next_waypoint(self):
        if not self.reference_collected or self.pause_timer: return
        if not self.current_trajectory:
            if self.next_state_after_motion:
                self.state, self.next_state_after_motion = self.next_state_after_motion, None
                self.execute_state()
            return
        target = self.current_trajectory.pop(0)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target[:3]
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = target[3:]
        self.cp_pub.publish(msg)
        self.last_command = target

def main(args=None):
    rclpy.init(args=args)
    node = PalpationGrid()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.logfile.flush()
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()