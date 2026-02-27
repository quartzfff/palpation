import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import numpy as np
import sys

class PalpationKuka(Node):
    def __init__(self):
        super().__init__('palpation_kuka')

        # --- CONFIGURATION ---
        self.use_relative_ndi = False 
        self.push_depth = 0.006       # 6mm push
        self.retract_lift = 0.005     # 5mm lift above surface
        self.push_direction = np.array([0.0, 0.0, 1.0]) 

        # --- TOPICS ---
        self.kuka_pub = self.create_publisher(PoseStamped, '/ves/kuka/servo_cp', 10)
        self.ves_pub = self.create_publisher(PoseStamped, '/ves/left/joint/servo_cp', 10)
        
        self.ndi_sub = self.create_subscription(PoseStamped, '/sensor_pose_raw', self.ndi_callback, 10)
        self.kuka_sub = self.create_subscription(PoseStamped, '/ves/kuka/measured_cp', self.kuka_callback, 10)

        # --- STATE MACHINE ---
        self.state = "MOVE_VES_INIT" 
        self.point_index = 0
        self.recording = False
        self.current_label = ""
        
        # --- DATA BUFFERS ---
        self.last_measured_kuka = None
        self.kuka_home_pos = None
        self.kuka_home_ori = None
        self.last_cmd_pose = None
        self.ndi_ref = None
        self.ref_samples = []
        self.kuka_wait_notified = False 
        
        # Grid points (Relative to locked KUKA origin)
        self.grid_points = [
            {'x': -0.008, 'y': 0.000, 'z': 0.008},
            {'x': -0.005, 'y': -0.00, 'z': 0.008},
            {'x': -0.002, 'y': -0.00, 'z': 0.008},
            {'x': 0.001, 'y': 0.000, 'z': 0.008},
            {'x': 0.005, 'y': -0.00, 'z': 0.008},
            {'x': 0.008, 'y': -0.00, 'z': 0.008}
        ]

        # CSV Logging Setup
        self.logfile = open('kuka_palpation_python.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.logfile)
        self.csv_writer.writerow(['time', 'point_index', 'phase', 'cmd_px', 'cmd_py', 'cmd_pz', 'ndi_px', 'ndi_py', 'ndi_pz'])

        # Timer (2.0s per step)
        self.timer = self.create_timer(2.0, self.state_machine)
        self.get_logger().info("🚀 Node Started. Moving VES to (0,0,0) first...")

    def kuka_callback(self, msg):
        self.last_measured_kuka = msg.pose

    def ndi_callback(self, msg):
        pos = msg.pose.position
        
        # Phase 1: Wait for VES to settle, then verify NDI signal and KUKA position
        if self.state == "WAITING_FOR_NDI":
            count = len(self.ref_samples)
            if count < 10:
                self.ref_samples.append([pos.x, pos.y, pos.z])
                if count % 2 == 0:
                    self.get_logger().info(f"📡 Collecting NDI samples: {count+1}/10")
                return

            # Once 10 NDI samples exist, check if KUKA measurement has arrived
            if self.last_measured_kuka:
                self.ndi_ref = np.mean(self.ref_samples, axis=0)
                self.kuka_home_pos = self.last_measured_kuka.position
                self.kuka_home_ori = self.last_measured_kuka.orientation
                
                self.get_logger().info("------------------------------------------")
                self.get_logger().info("✅ INITIALIZATION COMPLETE")
                self.get_logger().info(f"📍 NDI Baseline: {self.ndi_ref}")
                self.get_logger().info(f"🤖 KUKA Origin Locked.")
                self.get_logger().info("------------------------------------------")
                
                self.state = "COLLECT_BASELINE"
            else:
                if not self.kuka_wait_notified:
                    self.get_logger().warn("⏳ NDI Ready. Waiting for KUKA topic /ves/kuka/measured_cp...")
                    self.kuka_wait_notified = True
            return

        if not self.recording: return

        # Phase 2: Logging during grid execution
        t = self.get_clock().now().nanoseconds * 1e-9
        log_x = (pos.x - self.ndi_ref[0]) if self.use_relative_ndi else pos.x
        log_y = (pos.y - self.ndi_ref[1]) if self.use_relative_ndi else pos.y
        log_z = (pos.z - self.ndi_ref[2]) if self.use_relative_ndi else pos.z

        self.csv_writer.writerow([
            t, self.point_index, self.current_label,
            self.last_cmd_pose.position.x, self.last_cmd_pose.position.y, self.last_cmd_pose.position.z,
            log_x, log_y, log_z
        ])

    def get_target_msg(self, dx, dy, dz):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ves/kuka/base"
        msg.pose.position.x = self.kuka_home_pos.x + dx
        msg.pose.position.y = self.kuka_home_pos.y + dy
        msg.pose.position.z = self.kuka_home_pos.z + dz
        msg.pose.orientation = self.kuka_home_ori
        self.last_cmd_pose = msg.pose
        return msg

    def state_machine(self):
        if self.state == "MOVE_VES_INIT":
            # Exact command mapping from the topic pub request
            ves_msg = PoseStamped()
            ves_msg.header.frame_id = 'world'
            ves_msg.header.stamp = self.get_clock().now().to_msg()
            ves_msg.pose.position.x = 0.0 
            ves_msg.pose.position.y = 0.0 
            ves_msg.pose.position.z = 0.0 
            ves_msg.pose.orientation.x = 0.0
            ves_msg.pose.orientation.y = 0.0
            ves_msg.pose.orientation.z = 0.0
            ves_msg.pose.orientation.w = 1.0
            self.ves_pub.publish(ves_msg)
            
            self.get_logger().info("📦 VES positioned to (0,0,0). Settling for 3s...")
            self.ves_settle_start = self.get_clock().now()
            self.state = "VES_SETTLING"

        elif self.state == "VES_SETTLING":
            # Wait for mechanical vibrations to stop
            elapsed = (self.get_clock().now() - self.ves_settle_start).nanoseconds / 1e9
            if elapsed >= 3.0:
                self.get_logger().info("✅ VES Stable. Collecting NDI/KUKA origins...")
                self.state = "WAITING_FOR_NDI"

        elif self.state == "WAITING_FOR_NDI":
            return

        elif self.state == "COLLECT_BASELINE":
            self.start_record("baseline", True)
            self.state = "MOVE_TO_SURFACE"

        elif self.state == "MOVE_TO_SURFACE":
            self.recording = False
            pt = self.grid_points[self.point_index]
            self.kuka_pub.publish(self.get_target_msg(pt['x'], pt['y'], pt['z']))
            self.state = "PAUSE_SURFACE"

        elif self.state == "PAUSE_SURFACE":
            self.start_record("surface", True)
            self.state = "MOVE_TO_PUSH"

        elif self.state == "MOVE_TO_PUSH":
            self.recording = False
            pt = self.grid_points[self.point_index]
            px = pt['x'] + (self.push_depth * self.push_direction[0])
            py = pt['y'] + (self.push_depth * self.push_direction[1])
            pz = pt['z'] + (self.push_depth * self.push_direction[2])
            self.kuka_pub.publish(self.get_target_msg(px, py, pz))
            self.state = "PAUSE_PUSH"

        elif self.state == "PAUSE_PUSH":
            self.start_record("push", True)
            self.state = "RETRACT"

        elif self.state == "RETRACT":
            self.recording = False
            pt = self.grid_points[self.point_index]
            # Safety lift 5mm above surface
            self.kuka_pub.publish(self.get_target_msg(pt['x'], pt['y'], pt['z'] - self.retract_lift))
            self.state = "ADVANCE_POINT"

        elif self.state == "ADVANCE_POINT":
            self.point_index += 1
            if self.point_index >= len(self.grid_points):
                self.state = "DONE"
                self.get_logger().info("✅ Sequence Complete.")
            else:
                self.state = "MOVE_TO_SURFACE"

    def start_record(self, label, rec):
        self.current_label = label
        self.recording = rec
        self.get_logger().info(f"▶️ Phase: {label} | Point: {self.point_index}")

def main():
    rclpy.init()
    node = PalpationKuka()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.logfile.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()