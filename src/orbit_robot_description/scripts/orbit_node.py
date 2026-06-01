#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import time
from ikpy.chain import Chain

class OrbitTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('orbit_trajectory_publisher')
        
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        
        urdf_path = "/home/sam/orbit_sim_ws/src/orbit_robot_description/urdf/fairino10_v6.urdf"
        self.robot_chain = Chain.from_urdf_file(urdf_path)
        
        # Initialize a memory seed with zeros for all joints.
        # This keeps the math continuous and stops the "shaking".
        self.last_joint_angles = [0.0] * len(self.robot_chain.links)
        
        # Orbit Parameters (Adjust these carefully to stay within arm's reach)
        self.center_x = 0.4   
        self.center_y = 0.0   
        self.center_z = 0.5   
        self.radius = 0.12    
        self.speed = 0.3      
        
        self.timer = self.create_timer(0.02, self.timer_callback) 
        self.start_time = time.time()

    def timer_callback(self):
        elapsed = time.time() - self.start_time
        theta = self.speed * elapsed
        
        target_x = self.center_x + self.radius * np.cos(theta)
        target_y = self.center_y + self.radius * np.sin(theta)
        target_z = self.center_z  
        
        target_vector = [target_x, target_y, target_z]
        
        # CRITICAL FIX: We pass initial_position=self.last_joint_angles.
        joint_angles = self.robot_chain.inverse_kinematics(
            target_vector, 
            initial_position=self.last_joint_angles
        )
        
        # Save this solution to use as the seed for the next loop cycle
        self.last_joint_angles = joint_angles
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Fairino 10 typical joint layout 
        msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        
        # Safely slice active joints based on your link length
        msg.position = list(joint_angles[1:7]) 
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OrbitTrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()