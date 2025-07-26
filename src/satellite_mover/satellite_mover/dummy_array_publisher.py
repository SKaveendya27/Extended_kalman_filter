import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time
import tf_transformations 
import numpy as np

class DummyPublisher(Node):
    def __init__(self):
        super().__init__('dummy_publisher')
        # Publisher for Float64MultiArray on '/camera/odom'
        self.publisher_ = self.create_publisher(Float64MultiArray, '/camera/odom', 10)
        # Timer to publish at 10 Hz (every 0.1 seconds)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0  # Time parameter for trajectory
        self.get_logger().info('Dummy publisher node initialized')

        self.initx = 0.01  # Initial x position
        self.inity = 1.5  # Initial y position
        self.initz = 8.0  # Initial z position
        self.init_roll = -1.5708
        self.init_pitch = 0.0
        self.init_yaw = 0.0
        self.initxv = -0.1  # Initial x velocity
        self.initzv = 0.05
        self.init_rollv = 0.1  # Initial roll velocity

        # Noise standard deviation (0.1 for position in meters, orientation in radians)
        self.stddev = 0.05

    def timer_callback(self):
        msg = Float64MultiArray()
        # Generate a simple trajectory in the xy-plane with fixed height
        x = self.initx+(self.initxv*self.t) + np.random.normal(0.0, self.stddev)
        y = self.inity + np.random.normal(0.0, self.stddev)
        z = self.initz +(self.initzv*self.t) + np.random.normal(0.0, self.stddev)
        # Generate Euler angles (yaw, pitch, roll) in radians
        roll = self.init_roll+(self.init_rollv*self.t) + np.random.normal(0.0, self.stddev)# roll: 0.5 rad/s
        pitch = self.init_pitch + np.random.normal(0.0, self.stddev) #self.t * 0.2         
        yaw = self.init_yaw + np.random.normal(0.0, self.stddev)        # No yaw
        # Convert Euler angles to quaternion using tf_transformations
        quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)  # ZYX convention
        rx, ry, rz, w = quaternion
        # Timestamp in seconds
        time_now = time.time()
        # Pack data into array: [x, y, z, qx, qy, qz, qw, time]
        msg.data = [x, y, z, rx, ry, rz, w, time_now]
        # Publish message
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published: x={x:.2f}, y={y:.2f}, z={z:.2f}, '
            f'qx={rx:.2f}, qy={ry:.2f}, qz={rz:.2f}, qw={w:.2f}, time={time_now:.2f}'
        )
        # Increment time parameter
        self.t += 0.1

def main(args=None):
    rclpy.init(args=args)
    node = DummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down dummy publisher')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 