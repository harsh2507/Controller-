#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf_transformations import euler_from_quaternion
from my_robot_interfaces.srv import NextGoal
import time

class NextGoalServiceClient(Node):

    def __init__(self):
        super().__init__('next_goal_service_client')

        self.client = self.create_client(NextGoal, 'next_goal')

        self.req = NextGoal.Request()

    def send_request(self, index):
        self.req.request_goal = index

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        future = self.client.call_async(self.req)
        future.add_done_callback(self.on_response)

    def on_response(self, future):
        response = future.result()

        if response.end_of_list == 1:
            self.get_logger().info('Reached the end of the goal list.')

class HBTask1BController(Node):

    def __init__(self):
        super().__init__('hb_task1b_controller')

        # Initialize Publisher and Subscriber
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)

        # Initialize a Twist message to publish control velocities
        self.vel = Twist()

        # Initialize variables to track the robot's pose
        self.hb_x = 0
        self.hb_y = 0
        self.hb_theta = 0

        # Initialize other variables as needed
        self.rate = self.create_rate(100)  # Control loop rate

        # Initialize goal-related variables
        self.x_goals = [1, -1, -1, 1, 0]  # Example goal x-coordinates
        self.y_goals = [1, 1, -1, -1, 0]  # Example goal y-coordinates
        self.theta_goals = [0, math.pi/2, -math.pi, -math.pi/2, 0]  # Example goal yaw angles
        self.index = 0  # Current goal index
        self.flag = 0   # Flag to indicate the end of the goal list

        # Initialize P controller gains (adjust as needed)
        self.kp_linear = 0.5
        self.kp_angular = 0.5

        # Create a NextGoalServiceClient instance
        self.next_goal_service_client = NextGoalServiceClient()

        # Variables to track staying at the goal
        self.goal_reached_time = 0  # Time when goal was reached
        self.stay_duration = 1.0  # Minimum time to stay at a goal (adjust as needed)

    def odometry_callback(self, msg):
        # Extract and update the robot's pose from the odometry message
        self.hb_x = msg.pose.pose.position.x
        self.hb_y = msg.pose.pose.position.y

        # Convert the quaternion to Euler angles to get the orientation in radians
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.hb_theta = yaw

    def control_loop(self):
        while rclpy.ok():
            error_x = self.x_goals[self.index] - self.hb_x
            error_y = self.y_goals[self.index] - self.hb_y
            error_theta = self.theta_goals[self.index] - self.hb_theta

            self.vel.linear.x = self.kp_linear * error_x
            self.vel.linear.y = self.kp_linear * error_y
            self.vel.angular.z = self.kp_angular * error_theta

            self.limit_velocities()
            self.publisher.publish(self.vel)

            self.rate.sleep()

            if self.goal_reached(error_x, error_y, error_theta):
                if time.time() - self.goal_reached_time >= self.stay_duration:
                    if self.flag != 1:
                        self.index += 1
                    self.send_request(self.index)
                    self.goal_reached_time = time.time()

    def goal_reached(self, error_x, error_y, error_theta):
        # Define error thresholds for position and orientation (adjust as needed)
        position_threshold = 0.1  # Maximum allowed position error in meters
        orientation_threshold = 0.1  # Maximum allowed orientation error in radians

        # Check if the robot's position and orientation errors are within the thresholds
        position_error = math.sqrt(error_x**2 + error_y**2)

        return position_error < position_threshold and abs(error_theta) < orientation_threshold

    def limit_velocities(self):
        # Define maximum linear and angular velocities here (adjust as needed)
        max_linear_velocity = 1.0  # Maximum linear velocity in m/s
        max_angular_velocity = 1.0  # Maximum angular velocity in rad/s

        # Limit linear velocity
        if abs(self.vel.linear.x) > max_linear_velocity:
            self.vel.linear.x = max_linear_velocity
        if abs(self.vel.linear.y) > max_linear_velocity:
            self.vel.linear.y = max_linear_velocity

        # Limit angular velocity
        if abs(self.vel.angular.z) > max_angular_velocity:
            self.vel.angular.z = max_angular_velocity

def main(args=None):
    rclpy.init(args=args)

    # Create an instance of the HBTask1BController class
    ebot_controller = HBTask1BController()

    # Send an initial request with the index from ebot_controller.index
    ebot_controller.next_goal_service_client.send_request(ebot_controller.index)

    # Main loop
    while rclpy.ok():
        ebot_controller.control_loop()

        ############     DO NOT MODIFY THIS       #########
        ebot_controller.index += 1
        if ebot_controller.flag == 1:
            ebot_controller.index = 0
        ebot_controller.send_request(ebot_controller.index)
        ####################################################

        # Spin once to process callbacks
        rclpy.spin_once(ebot_controller)

    # Destroy the node and shut down ROS
    ebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
