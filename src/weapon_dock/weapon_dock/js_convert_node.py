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
        
        self.get_logger().info('JsConvertNode has been started.')
        
        self.linearx_scale = 2.0  # 线速度x缩放因子
        self.lineary_scale = 2.0 #  线性度y缩放因子
        self.angularz_scale = 2.0  # 角速度缩放因子
        self.dead_zone = 0.1  # 死区阈值


    def joy_callback(self, msg):
        # 检验是否有数据
        if not msg.axes and not msg.buttons:
            self.get_logger().warn('Received Joy message with no data')
            return

        # debug发出来具体接收到什么的信息
        # self.get_logger().debug(f'Received Joy details: axes={msg.axes}, buttons={msg.buttons}')

        twist = Twist()
        axes = msg.axes
        if len(axes) > 1:
            if abs(axes[1]) < self.dead_zone:
                twist.linear.x = 0.0
            else:
                twist.linear.x = axes[1] * self.linearx_scale 
        else:
            twist.linear.x = 0.0


        if len(axes) > 2:
            if abs(axes[0]) < self.dead_zone:
                twist.linear.y = 0.0
            else:
                twist.linear.y = axes[0] * self.lineary_scale
        else:
            twist.linear.y = 0.0

        if len(axes) > 3:
            if abs(axes[3]) < self.dead_zone:
                twist.angular.z = 0.0
            else:
                twist.angular.z = axes[3] * self.angularz_scale
        else:
            twist.angular.z = 0.0
            
        self.publisher_.publish(twist)
       
        self.get_logger().debug(f'Published cmd_vel: linear.x={twist.linear.x}, linear.y={twist.linear.y}, angular.z={twist.angular.z}') 

def main(args=None):

    rclpy.init(args=args)
    node = JsConvertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()