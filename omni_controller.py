import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
import math

class OmniKinematics(Node):
    def __init__(self):
        super().__init__('omni_kinematics_node')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.listener_callback, 10)
        self.pub = self.create_publisher(Float64MultiArray, 'joint_group_velocity_controller/commands', 10)

    def listener_callback(self, msg):
        # Input mapping
        vx_final = -msg.linear.x
        vy_final = -msg.linear.y
        w = -msg.angular.z

        # 3-Wheel Omniwheel Inverse Kinematics (M1 at 90 degrees)
        res_m1 = vy_final + w
        res_m2 = -0.5 * vy_final - (math.sqrt(3)/2) * vx_final + w
        res_m3 = -0.5 * vy_final + (math.sqrt(3)/2) * vx_final + w

        output = Float64MultiArray()
        output.data = [float(res_m1), float(res_m2), float(res_m3)]
        self.pub.publish(output)

def main(args=None):
    rclpy.init(args=args)
    node = OmniKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
