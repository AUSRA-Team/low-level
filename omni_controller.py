import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class OmniKinematics(Node):
    def __init__(self):
        super().__init__('omni_kinematics_node')
        # Listen to keyboard
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        # Talk to ESP32
        self.pub = self.create_publisher(Float64MultiArray, 'joint_group_velocity_controller/commands', 10)

    def listener_callback(self, msg):
        vx = msg.linear.x
        vy = msg.linear.y
        w  = msg.angular.z  # Rotation (Yaw)

        # Updated Kinematics for Front-Centered Motor (m1)
        # m1 is at the front (90 degrees)
        # m2 is back-left (210 degrees)
        # m3 is back-right (330 degrees)
        
        m1 = vy + w
        m2 = -0.5 * vy - (math.sqrt(3)/2) * vx + w
        m3 = -0.5 * vy + (math.sqrt(3)/2) * vx + w

        # To fix the "backward" issue: 
        # If the robot still goes backward when you press "i", 
        # we flip the signs of vx and vy globally:
        vx_final = -vx
        vy_final = vy
        
        # Applying the inverse kinematics matrix
        # m1 (Front) responds mostly to lateral (vy) or rotation
        # m2 and m3 handle the forward/backward drive (vx)
        res_m1 = vy_final + w
        res_m2 = -0.5 * vy_final - (math.sqrt(3)/2) * vx_final + w
        res_m3 = -0.5 * vy_final + (math.sqrt(3)/2) * vx_final + w

        output = Float64MultiArray()
        output.data = [float(res_m1), float(res_m2), float(res_m3)]
        self.pub.publish(output)
        
def main(args=None):
    rclpy.init(args=args)
    node = OmniKinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
