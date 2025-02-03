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

        # Waypoint params
        self.waypoints = np.array([[10, 5], [44, 12]])
        self.waypoint_index = 0
        self.switch_waypoint_distance = 5.0

        # State: x, y, psi, u, v, r
        self.state = None

        # APF parameters
        self.lidar_max_distance = 10.0
        self.lidar_angles = np.linspace(-np.pi/2, np.pi/2, 32) # 32 beams, must match lidar_publisher.py
        self.attractive_force_gain = 3.0

        # Controller parameters
        self.prev_heading_error = 0.0
        self.kp = 1.0
        self.kd = 0.1
        self.dt = 0.5 # seconds, should it be fixed?
        self.base_thrust = 2.5
        self.max_thrust = 10.0
        self.min_thrust = -10.0

    def control_callback(self, lidar_msg, state_msg):
        """Callback for synchronized Lidar and state data."""
        self.get_logger().info(f'Received synchronized data: Lidar={lidar_msg.data}, State={state_msg.data}')
        # thrust = np.random.uniform(0.0, 1.0, 2).tolist()

        self.state = state_msg.data

        psi_d = self.LOS_guidance()
        psi_d = self.apf_obstacle_avoidance(lidar_msg.data, psi_d)
        thrust_diff = self.pd_controller(psi_d)
        thrust = self.force_allocation(thrust_diff)

        msg = Float32MultiArray()
        msg.data = thrust.tolist()
        self.thrust_publisher_.publish(msg)
        self.get_logger().info(f'Publishing thrust: "{msg.data}"')

    def LOS_guidance(self):
        """Line-of-sight guidance algorithm."""
        if self.state is None:
            return
        if self.waypoint_index >= len(self.waypoints):
            return self.state[2] # Return current heading if all waypoints reached
        
        x, y, psi, u, v, r = self.state

        x_d, y_d = self.waypoints[self.waypoint_index]
        dx = x_d - x
        dy = y_d - y
        psi_d = np.arctan2(dy, dx) # Desired heading

        if np.sqrt(dx**2 + dy**2) < self.switch_waypoint_distance:
            self.waypoint_index += 1
            if self.waypoint_index < len(self.waypoints):
                return self.LOS_guidance()            
        return psi_d
            

    def apf_obstacle_avoidance(self, lidar_data, psi_d):
        """Artificial potential field obstacle avoidance algorithm."""
        repulsive_force = np.array([0.0, 0.0])

        for dist, angle in zip(lidar_data, self.lidar_angles):
            if dist < self.lidar_max_distance:
                if 0 < dist:
                    repulsive_force += (1.0 / dist**2) * np.array([-np.cos(angle), -np.sin(angle)])
                else:
                    repulsive_force += np.array([-np.cos(angle), -np.sin(angle)])

        attractive_force = self.attractive_force_gain * np.array([np.cos(psi_d), np.sin(psi_d)])
        total_force = attractive_force + repulsive_force
        psi_d_new = np.arctan2(total_force[1], total_force[0])
        return psi_d_new
                    


    def pd_controller(self, psi_d):
        """Proportional-derivative controller."""
        
        if self.state is None:
            return 0.0

        x, y, psi, u, v, r = self.state
        psi_error = psi_d - psi
        psi_error = np.arctan2(np.sin(psi_error), np.cos(psi_error))

        psi_error_dot = psi_error - self.prev_heading_error
        self.prev_heading_error = psi_error

        thrust_diff = self.kp * psi_error + self.kd * psi_error_dot
        return np.clip(thrust_diff, self.min_thrust, self.max_thrust)


    def force_allocation(self, thrust_diff):
        """Force allocation algorithm. Should be changed to PWM signals."""
        T_left = self.base_thrust + thrust_diff / 2
        T_right = self.base_thrust - thrust_diff / 2

        T_left = np.clip(T_left, self.min_thrust, self.max_thrust)
        T_right = np.clip(T_right, self.min_thrust, self.max_thrust)

        return np.array([T_left, T_right])

def main(args=None):
    rclpy.init(args=args)
    boat_controller = BoatController()
    rclpy.spin(boat_controller)
    boat_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
