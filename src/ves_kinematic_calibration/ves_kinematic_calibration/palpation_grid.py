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
            float(p[0]),
            float(p[1]),
            float(p[2]),
            0.0, 0.0, 0.0, 1.0
        ])
    return waypoints


def normalize(v):
    v = np.array(v, dtype=float)
    return v / (np.linalg.norm(v) + 1e-12)


# --------------------------------------------------
# Palpation Node
# --------------------------------------------------

class PalpationGrid(Node):

    def __init__(self):
        super().__init__('palpation_grid')

        # Publishers / Subscribers
        self.cp_pub = self.create_publisher(
            PoseStamped,
            '/ves/left/joint/servo_cp',
            10
        )

        self.ndi_sub = self.create_subscription(
            PoseStamped,
            '/sensor_pose_raw',
            self.ndi_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointState,
            '/ves/left/joint/setpoint_jp',
            self.joint_callback,
            10
        )

        self.current_joints = [0.0] * 5

        # Parameters
        self.publish_rate = 50.0
        self.transition_time = 2.0
        self.pause_time = 3.0
        self.home_pause_time = 2.0
        self.baseline_time = 2.0

        self.push_depth = 0.006
        self.push_direction = normalize([-0.2, -0.8, 0.0])

        # Zeroing
        self.reference_collected = False
        self.reference_samples = []
        self.reference_ndi = None
        self.get_logger().info("⌛ Collecting 10 NDI reference samples...")

        # Poses
        self.home_pose = [0.0, 0.0, 0.0, 0, 0, 0, 1]

        self.grid_points = [
            # [-0.005, -0.004, 0.008, 0, 0, 0, 1],
            # [-0.003, -0.004, 0.008, 0, 0, 0, 1],
            # [0.000, -0.005, 0.008, 0, 0, 0, 1],
            # [0.003, -0.006, 0.008, 0, 0, 0, 1],
            # [0.005, -0.007, 0.008, 0, 0, 0, 1],


            # [-0.005, -0.004, 0.012, 0, 0, 0, 1],
            # [-0.003, -0.004, 0.012, 0, 0, 0, 1],
            # [0.000, -0.006, 0.012, 0, 0, 0, 1],
            # [0.003, -0.007, 0.012, 0, 0, 0, 1],
            # [0.005, -0.008, 0.012, 0, 0, 0, 1],

            # [-0.005, -0.004, 0.017, 0, 0, 0, 1],
            # [-0.003, -0.004, 0.017, 0, 0, 0, 1],
            # [0.000, -0.006, 0.017, 0, 0, 0, 1],
            # [0.003, -0.007, 0.017, 0, 0, 0, 1],
            # [0.005, -0.008, 0.017, 0, 0, 0, 1],


            # [-0.005, -0.004, 0.021, 0, 0, 0, 1],
            # [-0.003, -0.004, 0.021, 0, 0, 0, 1],
            # [0.000, -0.006, 0.021, 0, 0, 0, 1],
            # [0.003, -0.007, 0.021, 0, 0, 0, 1],
            # [0.005, -0.008, 0.021, 0, 0, 0, 1],



            # [-0.005, -0.003, 0.025, 0, 0, 0, 1],
            # [-0.003, -0.003, 0.025, 0, 0, 0, 1],
            # [0.000, -0.005, 0.025, 0, 0, 0, 1],
            # [0.003, -0.006, 0.025, 0, 0, 0, 1],
            # [0.005, -0.007, 0.025, 0, 0, 0, 1],


            [-0.008, -0.000, 0.008, 0, 0, 0, 1],
            [-0.005, -0.001, 0.008, 0, 0, 0, 1],
            [-0.002, -0.006, 0.008, 0, 0, 0, 1],
            [0.002, -0.009, 0.008, 0, 0, 0, 1],

            [-0.008, -0.000, 0.012, 0, 0, 0, 1],
            [-0.005, -0.002, 0.012, 0, 0, 0, 1],
            [-0.001, -0.004, 0.012, 0, 0, 0, 1],
            [0.001, -0.008, 0.012, 0, 0, 0, 1],

            [-0.009, -0.001, 0.016, 0, 0, 0, 1],
            [-0.005, -0.001, 0.016, 0, 0, 0, 1],
            [-0.001, -0.004, 0.016, 0, 0, 0, 1],
            [0.000, -0.007, 0.016, 0, 0, 0, 1],

            [-0.009, -0.001, 0.020, 0, 0, 0, 1],
            [-0.005, -0.001, 0.020, 0, 0, 0, 1],
            [-0.001, -0.004, 0.020, 0, 0, 0, 1],
     

        ]

        # State
        self.point_index = 0
        self.state = "WAITING_FOR_ZERO"

        self.current_trajectory = []
        self.last_command = self.home_pose.copy()
        self.next_state_after_motion = None

        self.recording = False
        self.current_label = ""
        self.pause_timer = None

        # CSV
        self.logfile = open('pbh_palpation_grid_002_anglepush.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)

        self.csv_writer.writerow([
            'time',
            'point_index',
            'phase',
            'cmd_px','cmd_py','cmd_pz',
            'cmd_ox','cmd_oy','cmd_oz','cmd_ow',
            'ndi_px_rel','ndi_py_rel','ndi_pz_rel',
            'ndi_ox','ndi_oy','ndi_oz','ndi_ow',
            'joint_0','joint_1','joint_2','joint_3','joint_4'
        ])

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_next_waypoint
        )

    # --------------------------------------------------

    def joint_callback(self, msg):
        joints = list(msg.position) if msg.position else []
        joints += [0.0] * (5 - len(joints))
        self.current_joints = joints[:5]

    # --------------------------------------------------

    def ndi_callback(self, msg):

        pos = msg.pose.position
        ori = msg.pose.orientation

        if not self.reference_collected:
            self.reference_samples.append(pos)
            if len(self.reference_samples) >= 10:
                self.reference_ndi = self.average_position(self.reference_samples)
                self.reference_collected = True
                self.get_logger().info("✅ Reference collected.")
                self.state = "GO_HOME_FOR_BASELINE"
                self.execute_state()
            return

        if not self.recording:
            return

        t = self.get_clock().now().nanoseconds * 1e-9

        self.csv_writer.writerow([
            t,
            self.point_index,
            self.current_label,
            *self.last_command,
            pos.x - self.reference_ndi[0],
            pos.y - self.reference_ndi[1],
            pos.z - self.reference_ndi[2],
            ori.x, ori.y, ori.z, ori.w,
            *self.current_joints
        ])

    # --------------------------------------------------

    def start_motion(self, start_cp, end_cp, next_state):
        self.current_trajectory = interpolate(
            start_cp,
            end_cp,
            dt=self.transition_time,
            rate=int(self.publish_rate)
        )
        self.next_state_after_motion = next_state

    def start_pause(self, label, duration, next_state, record):
        if self.pause_timer is not None:
            return

        self.current_label = label
        self.recording = record

        self.pause_timer = self.create_timer(
            duration,
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

    def execute_state(self):

        if self.state == "GO_HOME_FOR_BASELINE":
            self.start_motion(self.last_command, self.home_pose, "BASELINE_PAUSE")
            return

        if self.state == "BASELINE_PAUSE":
            self.get_logger().info("📊 Recording 2s baseline at home")
            self.start_pause("baseline", self.baseline_time,
                             "MOVE_TO_SURFACE", record=True)
            return

        if self.state == "MOVE_TO_SURFACE":
            surface_pose = self.grid_points[self.point_index]
            self.start_motion(self.last_command, surface_pose, "PAUSE_SURFACE")
            return

        if self.state == "PAUSE_SURFACE":
            self.get_logger().info(f"📍 Surface record point {self.point_index}")
            self.start_pause("surface", self.pause_time,
                             "MOVE_TO_PUSH", record=True)
            return

        if self.state == "MOVE_TO_PUSH":
            surface_pose = self.grid_points[self.point_index]
            p_surface = np.array(surface_pose[:3])
            p_push = p_surface + self.push_depth * self.push_direction
            push_pose = [float(p_push[0]), float(p_push[1]),
                         float(p_push[2]), 0, 0, 0, 1]
            self.start_motion(self.last_command, push_pose, "PAUSE_PUSH")
            return

        if self.state == "PAUSE_PUSH":
            self.get_logger().info(f"⬇️ Push record point {self.point_index}")
            self.start_pause("push", self.pause_time,
                             "RETRACT", record=True)
            return

        if self.state == "RETRACT":
            surface_pose = self.grid_points[self.point_index]
            self.start_motion(self.last_command, surface_pose,
                              "GO_HOME_AFTER_POINT")
            return

        if self.state == "GO_HOME_AFTER_POINT":
            self.start_motion(self.last_command, self.home_pose,
                              "PAUSE_HOME")
            return

        if self.state == "PAUSE_HOME":
            self.get_logger().info("🏠 Settling at home (no recording)")
            self.start_pause("home_settle", self.home_pause_time,
                             "ADVANCE_POINT", record=False)
            return

        if self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.grid_points):
                self.state = "DONE"
            else:
                self.state = "MOVE_TO_SURFACE"
            self.execute_state()
            return

        if self.state == "DONE":
            self.get_logger().info("✅ All palpation points complete.")
            return

    # --------------------------------------------------

    def publish_next_waypoint(self):

        if not self.reference_collected:
            return

        if self.pause_timer is not None:
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

    @staticmethod
    def average_position(positions):
        n = len(positions)
        return (
            sum(p.x for p in positions) / n,
            sum(p.y for p in positions) / n,
            sum(p.z for p in positions) / n
        )


def main(args=None):
    rclpy.init(args=args)
    node = PalpationGrid()
    try:
        rclpy.spin(node)
    finally:
        node.logfile.flush()
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
