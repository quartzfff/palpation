import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, WrenchStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
import numpy as np
import csv


def interpolate_waypoints(start, end, dt=3.0, rate=50):
    num_steps = int(dt * rate)
    t = np.linspace(0, 1, num_steps)
    smooth_t = t**2 * (3 - 2 * t)
    waypoints = np.outer(smooth_t, np.array(end) - np.array(start)) + np.array(start)
    return waypoints


class KinematicCalibrator(Node):
    def __init__(self):
        super().__init__('kinematic_calibrator')

        self.joint_pub = self.create_publisher(JointState, '/ves/left/joint/servo_jp', 10)

        self.measured_cp_sub = Subscriber(self, PoseStamped, '/ves/left/joint/setpoint_cp')
        self.ndi_pose_sub = Subscriber(self, PoseStamped, '/sensor_pose_raw')
        self.force_sub = Subscriber(self, WrenchStamped, '/netft_data')

        self.ts = ApproximateTimeSynchronizer(
            [self.measured_cp_sub, self.ndi_pose_sub, self.force_sub], queue_size=50, slop=0.05
        )
        self.ts.registerCallback(self.sync_callback)

        self.test_commands = [[0.0, 0.0, 0.0, 0.0, 0.0]]
        self.test_commands = [[0.0, 0.0, 0.02, 0.0, 0.0]]
        # self.test_commands.append([0.0, 5.17, 0.017, 0.0038, 0.0])
        # self.test_commands.append([0.003, 4.5, 0.017, 0.005, 0.0])

        # self.test_commands.append([0.0, 0.0, 0.0, 0.0, 0.0])
        # self.test_commands.append([0.0, 5.17, 0.017, 0.0038, 0.0])
        # self.test_commands.append([0.0, 5.7, 0.017, 0.005, 0])

        # self.test_commands.append([0.0, 0.0, 0.0, 0.0, 0.0])
        # self.test_commands.append([0.0, 5.17, 0.017, 0.0038, 0.0])
        # self.test_commands.append([0.0, 4.07, 0.017, 0.004, 0])

       
        #self.test_commands.append([3.01,4.76,0.018,0.0083, 0])
       

  

        # r = 0.03   # inner translation
        # z = 0.02   # outer translation
        # fifth = 0.0

        # # Sweep 0 → 360 degrees
        # for theta in np.arange(0, 2 * np.pi + 1e-6, np.pi / 6):  # +1e-6 ensures ros2 topic hz /ves/left/joint/setpoint_cp2π is included
        #     self.test_commands.append([0.0, theta, r, z, fifth])
        self.test_commands.append([0.0, 0.0, 0.0, 0.0, 0.0])
        self.command_index = -1
        self.last_command = [0.0] * 5
        self.current_trajectory = []

        self.reference_collected = False
        self.reference_samples = []
        self.reference_ves = None
        self.reference_ndi = None
        self.reference_force = None

        self.logfile = open('spring_2cm_10deg_alu2.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow([
            'time',
            'joint_0', 'joint_1', 'joint_2', 'joint_3', 'joint_4',
            'ves_px', 'ves_py', 'ves_pz',
            'ves_ox', 'ves_oy', 'ves_oz', 'ves_ow',
            'ndi_px', 'ndi_py', 'ndi_pz',
            'ndi_ox', 'ndi_oy', 'ndi_oz', 'ndi_ow',
            'fx', 'fy', 'fz'
        ])

        self.get_logger().info("⌛ Collecting reference (first 10 samples)...")

        self.publish_rate = 50       # Hz
        self.transition_time = 2.0   # seconds
        self.pause_time = 5.0        # seconds pause
        self.pause_timer = None      # for one-shot pause timer

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['ir', 'or', 'it', 'ot', 'tool']
        self.joint_state_msg.position = [0.0] * 5

        self.publish_timer = self.create_timer(1.0 / self.publish_rate, self.publish_next_waypoint)

    def sync_callback(self, measured_cp, ndi_pose, force):
        if not self.reference_collected:
            self.reference_samples.append((measured_cp, ndi_pose, force))
            self.get_logger().info(f"📡 Reference sample {len(self.reference_samples)}/10")
            if len(self.reference_samples) >= 10:
                self.reference_ves = self.average_pose([m.pose.position for m, _, _ in self.reference_samples])
                self.reference_ndi = self.average_pose([n.pose.position for _, n, _ in self.reference_samples])
                self.reference_force = self.average_force([f.wrench.force for _, _, f in self.reference_samples])
                self.reference_collected = True
                self.get_logger().info("✅ Reference collected. Starting motion...")
                self.send_next_command()
            return

        ves_pos = measured_cp.pose.position
        ves_ori = measured_cp.pose.orientation
        ndi_pos = ndi_pose.pose.position
        ndi_ori = ndi_pose.pose.orientation
        f = force.wrench.force

        t = self.get_clock().now().to_msg().sec

        row = [
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
            f.x - self.reference_force[0],
            f.y - self.reference_force[1],
            f.z - self.reference_force[2],
        ]

        self.csv_writer.writerow(row)

    def send_next_command(self):
        self.command_index += 1
        if self.command_index >= len(self.test_commands):
            self.get_logger().info("✅ All commands sent.")
            self.current_trajectory = []
            return

        next_command = self.test_commands[self.command_index]
        self.get_logger().info(f"➡️ Moving to command {self.command_index + 1}: {next_command}")

        self.current_trajectory = interpolate_waypoints(
            self.last_command, next_command,
            dt=self.transition_time,
            rate=self.publish_rate
        ).tolist()

        self.last_command = next_command

        # Reset pause timer tracking
        self.pause_timer = None

    def publish_next_waypoint(self):
        if not self.reference_collected:
            return

        if self.current_trajectory:
            point = self.current_trajectory.pop(0)
            self.joint_state_msg.position = point
            self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.joint_state_msg)
        elif self.command_index < len(self.test_commands) - 1:
            if self.pause_timer is None:
                self.get_logger().info(f"⏸️ Pausing {self.pause_time}s before next command...")
                self.pause_timer = self.create_timer(self.pause_time, self.send_next_command_once)

    def send_next_command_once(self):
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.pause_timer = None
        self.send_next_command()

    @staticmethod
    def average_pose(positions):
        n = len(positions)
        x = sum([p.x for p in positions]) / n
        y = sum([p.y for p in positions]) / n
        z = sum([p.z for p in positions]) / n
        return (x, y, z)

    @staticmethod
    def average_force(forces):
        n = len(forces)
        x = sum([f.x for f in forces]) / n
        y = sum([f.y for f in forces]) / n
        z = sum([f.z for f in forces]) / n
        return (x, y, z)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicCalibrator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
