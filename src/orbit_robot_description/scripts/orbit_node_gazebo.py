#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import time
from ikpy.chain import Chain

class GazeboOrbitPublisher(Node):
    def __init__(self):
        super().__init__('gazebo_orbit_publisher')
        
        # Target the ros2_control hardware trajectory gateway instead of raw frames
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        
        urdf_path = "/home/sam/orbit_sim_ws/src/orbit_robot_description/urdf/fairino10_v6.urdf"
        self.robot_chain = Chain.from_urdf_file(urdf_path)
        
        # Initialize seed arrays to guarantee spatial math continuity
        self.last_joint_angles = [0.0] * len(self.robot_chain.links)
        
        # Orbit Target Parameters
        self.center_x = 0.4
        self.center_y = 0.0
        self.center_z = 0.4
        self.radius = 0.12
        self.speed = 0.6  # Path speed tracker variable
        
        # Run loop at 20Hz (50ms interval) to provide smooth instructions for the physics solvers
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_time = time.time()
        self.get_logger().info("Gazebo Trajectory Controller active and monitoring...")

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        theta = self.speed * elapsed
        
        # 1. Map target position coordinates
        target_x = self.center_x + self.radius * np.cos(theta)
        target_y = self.center_y + self.radius * np.sin(theta)
        target_z = self.center_z
        
        # 2. Compute continuous path angles
        joint_angles = self.robot_chain.inverse_kinematics(
            [target_x, target_y, target_z], 
            initial_position=self.last_joint_angles
        )
        self.last_joint_angles = joint_angles
        
        # 3. Create the Trajectory message structure required by ros2_control
        msg = JointTrajectory()
        msg.joint_names = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles[1:7]) # Slices active structural joints
        
        # Allocate execution budget (Must closely match your control timer loop interval)
        point.time_from_start = Duration(sec=0, nanosec=50000000) # 50 milliseconds
        
        msg.points.append(point)
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GazeboOrbitPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()