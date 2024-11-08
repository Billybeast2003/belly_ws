import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill, Spawn
from math import pow, atan2, sqrt
import random
import time

class TurtleCatcher(Node):
    def __init__(self):
        super().__init__('turtle_catcher')

        self.position = Pose()
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.position_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.update_position, 10)
        self.client = self.create_client(Kill, '/kill')
        self.spawner()

        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Kill service is not available right now')
            return

        self.request = Kill.Request()
        self.request.name = "turtle_spawn"

            
    def spawner(self):
        theta_spawn = 0.0
        client = self.create_client(Spawn, '/spawn')

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Spawn service is not available right now')
            rclpy.shutdown()
            return

        self.spawn_request = Spawn.Request()
        self.spawn_request.name = "turtle_spawn"

        # Set random desired position x, y, and theta for the new spawner turtle
        self.spawn_request.x = random.uniform(1, 10)
        self.spawn_request.y = random.uniform(1, 10)
        self.spawn_request.theta = theta_spawn

        future = client.call_async(self.spawn_request)

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error('Error while calling the service: %s' % (str(e)))
            rclpy.shutdown()
            return

    def update_position(self, data):
        self.position.x = data.x
        self.position.y = data.y
        self.position.theta = data.theta
        msg = f"X: {data.x:.3f}, Y: {data.y:.3f}, Theta: {data.theta:.3f}"
        self.get_logger().info(msg)
        self.move2goal()

    def euclidean_distance(self, goal_position):
        return sqrt(pow((goal_position.x - self.position.x), 2) +
                    pow((goal_position.y - self.position.y), 2))

    def linear_velocity(self, goal_position, constant=1.5):
        return constant * self.euclidean_distance(goal_position)

    def staring_angle(self, goal_position):
        return atan2(goal_position.y - self.position.y, goal_position.x - self.position.x)

    def angular_velocity(self, goal_position, constant=5):
        return constant * (self.staring_angle(goal_position) - self.position.theta)

    def move2goal(self):
        goal_position = Pose()
        goal_position.x = self.spawn_request.x
        goal_position.y = self.spawn_request.y
        goal_position.theta = self.spawn_request.theta

        velocity_msg = Twist()

        if self.euclidean_distance(goal_position) >= 0.5:
            velocity_msg.angular.z = self.angular_velocity(goal_position)
            velocity_msg.linear.x = self.linear_velocity(goal_position)
        else:
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.0
            future = self.client.call_async(self.request)
            time.sleep(1)
            self.spawner()

        self.velocity_publisher.publish(velocity_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleCatcher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()