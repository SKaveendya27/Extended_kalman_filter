import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose

class MoveSatellite(Node):
    def __init__(self):
        super().__init__('move_satellite')
        self.current_pose = None

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/sat/pose',
            self.pose_callback,
            10
        )
    
    def pose_callback(self, pose):       #checking pose every 0.1 sec
        self.current_pose = pose
        self.timer = self.create_timer(0.1, self.move_satellite)

    def move_satellite(self):
        if not self.current_pose:
            return  # Wait until pose is received

        cmd = Twist()

        cmd.linear.z = -0.1 # Move forward in straight line
        # cmd.angular.z =  0.1  Rotate around the z-axis
        # cmd.angular.x = 0.5

        self.publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MoveSatellite()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
