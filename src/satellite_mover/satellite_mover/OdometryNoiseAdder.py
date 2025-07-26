import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
from tf2_py import quaternion_from_euler, euler_from_quaternion, quaternion_multiply

class OdometryNoiseAdder(Node):
    def __init__(self):
        super().__init__('odometry_noise_adder')
        # Declare and get use_sim_time parameter
        self.declare_parameter('use_sim_time', True)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().boolean_value

        # Noise standard deviation (0.1 for position in meters, orientation in radians)
        self.stddev = 10.0

        # Subscriber to /sat/odom
        self.subscription = self.create_subscription(
            Odometry,
            '/sat/odom',
            self.odom_callback,
            10
        )

        # Publisher to /sat/odom/updated
        self.publisher = self.create_publisher(
            Odometry,
            '/sat/odom/updated',
            10
        )

        self.get_logger().info('OdometryNoiseAdder node started, adding noise to /sat/odom and publishing to /sat/odom/updated')

    def odom_callback(self, msg):
        # Create a new odometry message
        noisy_odom = Odometry()
        noisy_odom.header = msg.header  # Copy header (timestamp, frame_id)
        noisy_odom.child_frame_id = msg.child_frame_id  # Copy child_frame_id

        # Add noise to position
        noisy_odom.pose.pose.position.x = msg.pose.pose.position.x + np.random.normal(0.0, self.stddev)
        noisy_odom.pose.pose.position.y = msg.pose.pose.position.y + np.random.normal(0.0, self.stddev)
        noisy_odom.pose.pose.position.z = msg.pose.pose.position.z + np.random.normal(0.0, self.stddev)

        # Add noise to orientation
        # Convert quaternion to Euler angles
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(quat)

        # Add noise to Euler angles
        noisy_roll = roll + np.random.normal(0.0, self.stddev)
        noisy_pitch = pitch + np.random.normal(0.0, self.stddev)
        noisy_yaw = yaw + np.random.normal(0.0, self.stddev)

        # Convert noisy Euler angles back to quaternion
        noisy_quat = quaternion_from_euler(noisy_roll, noisy_pitch, noisy_yaw)
        noisy_odom.pose.pose.orientation.x = noisy_quat[0]
        noisy_odom.pose.pose.orientation.y = noisy_quat[1]
        noisy_odom.pose.pose.orientation.z = noisy_quat[2]
        noisy_odom.pose.pose.orientation.w = noisy_quat[3]

        # Copy covariance (unchanged)
        noisy_odom.pose.covariance = msg.pose.covariance

        # Copy twist (velocity, unchanged)
        noisy_odom.twist = msg.twist

        # Publish noisy odometry
        self.publisher.publish(noisy_odom)
        self.get_logger().debug('Published noisy odometry to /sat/odom/updated')

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OdometryNoiseAdder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()