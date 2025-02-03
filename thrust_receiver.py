import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ThrustReceiver(Node):
    def __init__(self):
        super().__init__('thrust_receiver')
        self.subscription = self.create_subscription(Float32MultiArray, 'thrust', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    thrust_receiver = ThrustReceiver()
    rclpy.spin(thrust_receiver)
    thrust_receiver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()