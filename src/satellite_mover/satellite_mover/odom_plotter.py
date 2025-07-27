import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
from tf_transformations import euler_from_quaternion

class PoseTimeSeriesPlotter(Node):
    def __init__(self):
        super().__init__('pose_time_series_plotter')

        # Data stores: [x, y, z, roll, pitch, yaw, t]
        self.gt_data = {'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': [], 't': []}
        self.ekf_data = {'x': [], 'y': [], 'z': [], 'roll': [], 'pitch': [], 'yaw': [], 't': []}

        # Subscribers
        self.create_subscription(Odometry, 'sat/odom', self.gt_callback, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.ekf_callback, 10)

        # Timer to plot after 30 seconds
        self.create_timer(30.0, self.plot_all)

    def extract_pose(self, msg):
        # Position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Orientation (quaternion to RPY)
        q = msg.pose.pose.orientation
        norm = (q.x**2 + q.y**2 + q.z**2 + q.w**2)**0.5
        if norm < 1e-6:
            return None
        rpy = euler_from_quaternion([q.x / norm, q.y / norm, q.z / norm, q.w / norm])
        # Timestamp (as float seconds)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return x, y, z, rpy[0], rpy[1], rpy[2], t

    def gt_callback(self, msg):
        result = self.extract_pose(msg)
        if result:
            x, y, z, r, p, yaw, t = result
            self.gt_data['x'].append(x)
            self.gt_data['y'].append(y)
            self.gt_data['z'].append(z)
            self.gt_data['roll'].append(r)
            self.gt_data['pitch'].append(p)
            self.gt_data['yaw'].append(yaw)
            self.gt_data['t'].append(t)

    def ekf_callback(self, msg):
        result = self.extract_pose(msg)
        if result:
            x, y, z, r, p, yaw, t = result
            self.ekf_data['x'].append(x)
            self.ekf_data['y'].append(y)
            self.ekf_data['z'].append(z)
            self.ekf_data['roll'].append(r)
            self.ekf_data['pitch'].append(p)
            self.ekf_data['yaw'].append(yaw)
            self.ekf_data['t'].append(t)

    def plot_all(self):
        fig, axs = plt.subplots(6, 1, figsize=(12, 18), sharex=True)

        fields = ['x', 'y', 'z', 'roll', 'pitch', 'yaw']
        labels = ['X Position (m)', 'Y Position (m)', 'Z Position (m)',
                  'Roll (rad)', 'Pitch (rad)', 'Yaw (rad)']

        for i, field in enumerate(fields):
            ax = axs[i]

            if self.gt_data[field]:
                ax.plot(self.gt_data['t'], self.gt_data[field], label='GT', color='green')
            if self.ekf_data[field]:
                ax.plot(self.ekf_data['t'], self.ekf_data[field], label='EKF', color='blue')

            ax.set_ylabel(labels[i])
            ax.grid(True)
            ax.legend(loc='upper right')

        axs[-1].set_xlabel("Time (s)")
        plt.suptitle("Pose and Orientation over Time (GT vs Raw vs EKF)")
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        plt.show()

        self.get_logger().info("Plotted all data. Shutting down.")
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PoseTimeSeriesPlotter()
        rclpy.spin(node)
    except Exception as e:
        print(f"[ERROR] {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
