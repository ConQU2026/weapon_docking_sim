import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JsConvertNode(Node):
    def __init__(self):
        super().__init__('js_convert_node')
        # 订阅手柄指令
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # 发布速度指令
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def joy_callback(self, msg):
        twist = Twist()

        axes = msg.axes

        if len(key_list) > 1:
            twist.linear.x = key_list[1] * 0.5
        else:
            twist.linear.x = 0.0

        if len(key_list) > 3:
            twist.angular.z = key_list[3] * 1.0
        else:
            twist.angular.z = 0.0
        self.publisher_.publish(twist)
        
        self.get_logger().info(f'Published cmd_vel: linear.x={twist.linear.x}, angular.z={twist.angular.z}') 

def main(args=None):
    rclpy.init(args=args)
    node = JsConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()