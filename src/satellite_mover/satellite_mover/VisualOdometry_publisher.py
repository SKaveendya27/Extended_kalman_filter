import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Vector3, Twist, Point, Quaternion
import numpy as np

class VisualOdometryPublisher(Node):
    def __init__(self):
        super().__init__('visual_odometry_publisher')
        # Check if use_sim_time is already declared, otherwise declare it
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)  # Default to wall-clock time
        use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.get_logger().info(f'Using sim time: {use_sim_time}')

        # Subscriber to /camera/odom
        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/camera/odom',
            self.odom_callback,
            10
        )

        # Publisher to /camera/odom/updated
        self.publisher = self.create_publisher(
            Odometry,
            '/camera/odom/updated',
            10
        )
        self.prev_data = None
        self.prev_time = None
        self.get_logger().info('Visual odometry publisher node initialized')

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions q1 * q2."""
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return np.array([
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2
        ])

    def quaternion_conjugate(self, q):
        """Return conjugate of quaternion."""
        return np.array([-q[0], -q[1], -q[2], q[3]])

    def odom_callback(self, msg):
        # Validate input: expect 8 values [x, y, z, rx, ry, rz, w, time]
        if len(msg.data) != 8:
            self.get_logger().warn(f'Expected 8 values, got {len(msg.data)}')
            return
        
        x, y, z, rx, ry, rz, w, time = msg.data
        # Normalize quaternion to ensure valid rotation
        q_norm = np.sqrt(rx**2 + ry**2 + rz**2 + w**2)
        
        if q_norm == 0:
            self.get_logger().warn('Invalid quaternion: norm is zero')
            return
        rx, ry, rz, w = rx/q_norm, ry/q_norm, rz/q_norm, w/q_norm

        # Create a new odometry message
        camera_odom = Odometry()
        camera_odom.header.frame_id = 'satellite/odom'
        camera_odom.header.stamp = self.get_clock().now().to_msg()
        camera_odom.child_frame_id = 'satellite/sat_body'

        # Set pose
        camera_odom.pose.pose.position = Point(x=x, y=y, z=z)
        camera_odom.pose.pose.orientation = Quaternion(x=rx, y=ry, z=rz, w=w)
     
        # Compute velocities if previous data exists
        if self.prev_data is not None and self.prev_time is not None:
            dt = time - self.prev_time
            if dt <= 0:
                self.get_logger().warn(f'Invalid time difference: {dt} seconds')
                return
            self.get_logger().info(f'Time difference: {dt} seconds')
            
            # Linear velocities
            prev_x, prev_y, prev_z, prev_rx, prev_ry, prev_rz, prev_w = self.prev_data
            vx = (x - prev_x) / dt
            vy = (y - prev_y) / dt
            vz = (z - prev_z) / dt

            # Angular velocities from quaternion difference
            q_curr = np.array([rx, ry, rz, w])
            q_prev = np.array([prev_rx, prev_ry, prev_rz, prev_w])
            q_diff = self.quaternion_multiply(q_curr, self.quaternion_conjugate(q_prev))
            self.get_logger().info(f'Quaternion difference: {q_diff}')
            qx, qy, qz, qw = q_diff
            if abs(qw) > 1:
                qw = 1.0 if qw > 0 else -1.0
            angle = 2 * np.arccos(qw)
            if abs(angle) < 1e-6:
                wx, wy, wz = 0.0, 0.0, 0.0
            else:
                sin_half_angle = np.sqrt(qx**2 + qy**2 + qz**2)
                if sin_half_angle < 1e-6:
                    wx, wy, wz = 0.0, 0.0, 0.0
                else:
                    axis = np.array([qx, qy, qz]) / sin_half_angle
                    wx, wy, wz = axis * angle / dt

            # Set velocities in odometry message
            camera_odom.twist.twist = Twist()
            camera_odom.twist.twist.linear = Vector3(x=vx, y=vy, z=vz)
            camera_odom.twist.twist.angular = Vector3(x=wx, y=wy, z=wz)
            self.get_logger().info(f'Computed velocities: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, wx={wx:.2f}, wy={wy:.2f}, wz={wz:.2f}')
        
        # Publish odometry
        self.publisher.publish(camera_odom)

        # Store current data
        self.prev_data = (x, y, z, rx, ry, rz, w)

        self.prev_time = time

    def destroy_node(self):
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()