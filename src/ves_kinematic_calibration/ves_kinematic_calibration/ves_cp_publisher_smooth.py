import csv
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, WrenchStamped
from sensor_msgs.msg import JointState
from message_filters import Subscriber, ApproximateTimeSynchronizer

# --------------------------------------------------
# Utilities
# --------------------------------------------------

def smoothstep(n):
    t = np.linspace(0.0, 1.0, n)
    return t**2 * (3.0 - 2.0 * t)

def interpolate_cp_pos_only(start_cp, end_cp, dt=3.0, rate=50):
    num_steps = max(2, int(dt * rate))
    ts = smoothstep(num_steps)
    p0 = np.array(start_cp[:3], dtype=float)
    p1 = np.array(end_cp[:3], dtype=float)

    waypoints = []
    for t in ts:
        p = (1.0 - t) * p0 + t * p1
        waypoints.append([p[0], p[1], p[2], 0.0, 0.0, 0.0, 1.0])
    return waypoints

# --------------------------------------------------
# Node
# --------------------------------------------------

class KinematicCalibrator(Node):

    FORCE_BIAS_SAMPLES = 10

    def __init__(self):
        super().__init__('kinematic_calibrator')

        # Publisher
        self.cp_pub = self.create_publisher(
            PoseStamped, '/ves/left/joint/servo_cp', 10)

        # Subscribers
        self.measured_cp_sub    = Subscriber(self, PoseStamped, '/ves/left/joint/setpoint_cp')
        self.ndi_pose_sub       = Subscriber(self, PoseStamped, '/sensor_pose_raw')
        self.force_sub          = Subscriber(self, WrenchStamped, '/netft_data')
        self.joint_setpoint_sub = Subscriber(self, JointState, '/ves/left/joint/setpoint_jp')

        self.ts = ApproximateTimeSynchronizer(
            [
                self.measured_cp_sub,
                self.ndi_pose_sub,
                self.force_sub,
                self.joint_setpoint_sub
            ],
            queue_size=50,
            slop=0.02
        )
        self.ts.registerCallback(self.sync_callback)

        # Test commands
        self.test_commands = [

            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [0.00, 0.00, 0.016, 0.0, 0.0, 0.0, 1.0],
            # [0.00, 0.00, 0.019, 0.0, 0.0, 0.0, 1.0],
            # [0.00, 0.00, 0.016, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],

            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [0.00, 0.00, 0.016, 0.0, 0.0, 0.0, 1.0],
            # [0.006, 0.00, 0.019, 0.0, 0.0, 0.0, 1.0],
            # [0.00, 0.00, 0.016, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],



            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, 0.002, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, -0.004, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, 0.002, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],

            #right

            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [-0.003, 0.00, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [0.002, 0.00, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [-0.003, 0.00, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],

            #left

            # [0.00, 0.00, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [0.001, -0.001, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [-0.004, -0.001, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [0.001, -0.001, 0.02, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#bph test

            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, -0.011, 0.008, 0.0, 0.0, 0.0, 1.0],
            # [-0.001, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
            # [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


            [0.0, -0.003, 0.01, 0.0, 0.0, 0.0, 1.0],
            [0.0, -0.01, 0.01, 0.0, 0.0, 0.0, 1.0],
            [0.0, -0.003, 0.01, 0.0, 0.0, 0.0, 1.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.001, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.011, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.004, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.011, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.004, 0.008, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
# #center
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.011, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.0, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, -0.011, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.001, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.011, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.004, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.011, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.004, 0.01, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
#             # bot
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.011, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [-0.001, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.0, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, -0.011, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.001, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.011, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.001, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],


#             [0.004, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.011, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.004, -0.004, 0.013, 0.0, 0.0, 0.0, 1.0],
#             [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],



         ]

        self.command_index = -1
        self.last_command = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        self.current_trajectory = []

        # -------------------------------
        # Force bias (one-time)
        # -------------------------------
        self.force_bias_samples = []
        self.force_bias = None

        # -------------------------------
        # VES / NDI reference (optional)
        # -------------------------------
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ves = None
        self.reference_ndi = None

        # Logging
        self.logfile = open('jel_reg_001_old.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time',
            'cmd_px','cmd_py','cmd_pz','cmd_ox','cmd_oy','cmd_oz','cmd_ow',
            'ves_px','ves_py','ves_pz','ves_ox','ves_oy','ves_oz','ves_ow',
            'ndi_px','ndi_py','ndi_pz','ndi_ox','ndi_oy','ndi_oz','ndi_ow',
            'fx','fy','fz',
            'joint_0','joint_1','joint_2','joint_3','joint_4',
        ])

        self.get_logger().info("⌛ Collecting initial force bias (10 samples)...")

        # Timing
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 3.0
        self.pause_timer = None

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_next_waypoint)

    # --------------------------------------------------
    # Sync callback
    # --------------------------------------------------

    def sync_callback(self, measured_cp, ndi_pose, force, joint_state):

        f = force.wrench.force

        # ---- Force bias collection (once) ----
        if self.force_bias is None:
            self.force_bias_samples.append([f.x, f.y, f.z])
            self.get_logger().info(
                f"📡 Force bias sample {len(self.force_bias_samples)}/{self.FORCE_BIAS_SAMPLES}"
            )

            if len(self.force_bias_samples) >= self.FORCE_BIAS_SAMPLES:
                self.force_bias = np.mean(
                    np.array(self.force_bias_samples), axis=0)
                self.get_logger().info(
                    f"✅ Force bias latched: {self.force_bias}"
                )
            return

        # ---- VES / NDI reference (optional, same as before) ----
        if not self.reference_collected:
            self.reference_samples.append((measured_cp, ndi_pose))
            if len(self.reference_samples) >= 10:
                self.reference_ves = self.average_position(
                    [m.pose.position for m, _ in self.reference_samples])
                self.reference_ndi = self.average_position(
                    [n.pose.position for _, n in self.reference_samples])
                self.reference_collected = True
                self.get_logger().info("✅ Pose reference collected. Starting motion...")
                self.send_next_command()
            return

        # ---- Relative force (FIXED bias) ----
        fx = f.x - self.force_bias[0]
        fy = f.y - self.force_bias[1]
        fz = f.z - self.force_bias[2]

        ves_pos = measured_cp.pose.position
        ves_ori = measured_cp.pose.orientation
        ndi_pos = ndi_pose.pose.position
        ndi_ori = ndi_pose.pose.orientation

        joints = list(joint_state.position) if joint_state.position else []
        joints += [0.0] * (5 - len(joints))
        joints = joints[:5]

        t = self.get_clock().now().nanoseconds * 1e-9

        self.csv_writer.writerow([
            t,
            *self.last_command,
            ves_pos.x - self.reference_ves[0],
            ves_pos.y - self.reference_ves[1],
            ves_pos.z - self.reference_ves[2],
            ves_ori.x, ves_ori.y, ves_ori.z, ves_ori.w,
            ndi_pos.x - self.reference_ndi[0],
            ndi_pos.y - self.reference_ndi[1],
            ndi_pos.z - self.reference_ndi[2],
            ndi_ori.x, ndi_ori.y, ndi_ori.z, ndi_ori.w,
            fx, fy, fz,
            *joints,
        ])

    # --------------------------------------------------
    # Motion control
    # --------------------------------------------------

    def send_next_command(self):
        self.command_index += 1
        if self.command_index >= len(self.test_commands):
            self.get_logger().info("✅ All commands sent.")
            return

        next_cmd = self.test_commands[self.command_index]
        self.current_trajectory = interpolate_cp_pos_only(
            self.last_command, next_cmd,
            dt=self.transition_time,
            rate=int(self.publish_rate)
        )

        self.last_command = next_cmd
        self.pause_timer = None

    def publish_next_waypoint(self):
        if not self.reference_collected:
            return

        if self.current_trajectory:
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

        elif self.command_index < len(self.test_commands) - 1:
            if self.pause_timer is None:
                self.pause_timer = self.create_timer(
                    self.pause_time, self.send_next_command_once)

    def send_next_command_once(self):
        self.pause_timer.cancel()
        self.pause_timer = None
        self.send_next_command()

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------

    @staticmethod
    def average_position(positions):
        n = len(positions)
        return (
            sum(p.x for p in positions) / n,
            sum(p.y for p in positions) / n,
            sum(p.z for p in positions) / n
        )

# --------------------------------------------------
# Main
# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = KinematicCalibrator()
    try:
        rclpy.spin(node)
    finally:
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
