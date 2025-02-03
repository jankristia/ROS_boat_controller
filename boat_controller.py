import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import message_filters

class BoatController(Node):
    def __init__(self):
        super().__init__('boat_controller')

        # Use message_filters.Subscriber instead of create_subscription
        self.state_subscriber_ = message_filters.Subscriber(self, Float32MultiArray, 'state')
        self.lidar_subscriber_ = message_filters.Subscriber(self, Float32MultiArray, 'lidar')

        # Synchronizer: Approximate sync to handle small timing differences
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_subscriber_, self.state_subscriber_], queue_size=10, slop=1, allow_headerless=True,
        )
        self.sync.registerCallback(self.control_callback)

        self.thrust_publisher_ = self.create_publisher(Float32MultiArray, 'thrust', 10)


    def control_callback(self, lidar_msg, state_msg):
        """Callback for synchronized Lidar and state data."""
        self.get_logger().info(f'Received synchronized data: Lidar={lidar_msg.data}, State={state_msg.data}')
        thrust = np.random.uniform(0.0, 1.0, 2).tolist()

        msg = Float32MultiArray()
        msg.data = thrust
        self.thrust_publisher_.publish(msg)
        self.get_logger().info(f'Publishing thrust: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    boat_controller = BoatController()
    rclpy.spin(boat_controller)
    boat_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
