import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Pose

class MoveBox(Node):
    def __init__(self):
        super().__init__('move_box')
        self.current_pose = None
        self.target_distance = 2.0  # Distance to move in the straight line
        self.target_angle = math.pi / 2  # 90 degrees(rad)
        self.angular_speed = 10*2*math.pi/360
        self.current_angle = 0.0
        self.current_distance = 0.0

        # Initializing start_pose values
        self.start_pose = Pose()
        self.start_pose.position.x = 0.0
        self.start_pose.position.y = 0.0
        self.start_pose.orientation.y = 0.0
        self.t0 = time.time()

        self.is_turning = False  # True if the sat is currently turning

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # self.timer = self.create_timer(0.1, self.publish_velocity)
        self.subscription = self.create_subscription(
            Pose,
            '/box/pose',
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

        if self.is_turning:
            if self.current_angle>= self.target_angle :
                self.is_turning = False
                self.start_pose = self.current_pose  # Reset for straight movement
                self.current_angle = 0.0
                self.t0 = time.time()
            else:
                t1 = time.time()
                self.current_angle = self.angular_speed*(t1-self.t0)
                cmd.angular.x = self.angular_speed
                # cmd.angular.z = self.angular_speed
        else:
            if self.current_distance >= self.target_distance:
                self.is_turning = True
                self.start_pose = self.current_pose  # Reset for turning
                self.current_distance = 0.0
                self.t0 = time.time()
            else:
                t2 = time.time()
                self.current_distance = (t2-self.t0)
                cmd.linear.z = 1.0 # Move forward in straight line
                # cmd.angular.z = self.angular_speed

        self.publisher.publish(cmd)

    # def publish_velocity(self):
    #     msg = Twist()
    #     msg.linear.x = 1.0  # Move forward
    #     msg.linear.z = 1.0
    #     msg.angular.x = 1.0  # Rotate
    #     msg.angular.y = 0.5  # Rotate
    #     msg.angular.z = 0.5  # Rotate
    #     self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MoveBox()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
