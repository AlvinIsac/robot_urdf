#!/usr/bin/env python3
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2_aruco_interfaces.msg import ArucoMarkers
 
class FullRotationNode(Node):
    def __init__(self):
        super().__init__('full_rotation_node')
 
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.marker_sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.marker_callback, 10)
 
        self.rotation_speed = 4.0  # radians per second
        self.target_angle = 10 * 3.14159  # Approx. 360 degrees in radians
        self.current_angle = 0.0
      
        self.timer1 = self.create_timer(0.1, self.rotate)
        self.timer1_active = True  # Flag to control Timer 1
        self.timer2_callback_flag = False

        # Timer 2: Runs every 3 seconds
        self.timer2 = self.create_timer(3.0, self.timer2_callback)
        
        self.last_time = self.get_clock().now()
        
        # Initialize variables
        self.lowest_id = 500
        self.current_marker_id = 0
        self.GREEN = '\033[92m' 
        self.RESET = '\033[0m'
 
    def marker_callback(self, msg):
        self.current_marker_id = msg.marker_ids[-1]
        if msg.marker_ids[-1] < self.lowest_id:
            self.lowest_id = msg.marker_ids[-1]
            self.get_logger().warning(f"Lowest Id till now: {self.lowest_id}")

        return self.current_marker_id , self.lowest_id
 
    def rotate(self):
         if self.timer1_active:     
            current_time = self.get_clock().now()
            elapsed_time = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
    
            self.current_angle += self.rotation_speed * elapsed_time
    
            if self.current_angle >= self.target_angle:
                twist = Twist()
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                self.get_logger().error("Full 360 Rotataion Completed")
    
                if self.lowest_id is not None:
                    self.get_logger().info(f"{self.GREEN}Lowest ID: {self.lowest_id}.{self.RESET}")
                else:
                    self.get_logger().info("No markers detected during rotation.")
                
                self.timer1_active = False
                self.timer1.cancel()
                self.timer2.reset() 
                self.timer2_callback_flag = True
                return
            
            twist = Twist()
            twist.angular.z = self.rotation_speed
            self.cmd_vel_pub.publish(twist)
    
    def timer2_callback(self):
        if self.timer2_callback_flag:
            if self.lowest_id == self.current_marker_id:
                twist = Twist()
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist) 
                self.timer2.cancel()
                self.get_logger().info(f"{self.GREEN}I FOUND THE LOWEST MARKER STOPPING!!!.{self.RESET}")
                self.get_logger().info(f"{self.GREEN}ASSIGNMENT COMPLETED!!!.{self.RESET}")
                return
        
        self.get_logger().warning("Almost there.....")
        twist = Twist()
        twist.angular.z = -2.0
        self.cmd_vel_pub.publish(twist) 


def main(args=None):
    rclpy.init(args=args)
    node = FullRotationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()