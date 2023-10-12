#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import time


class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_Circle")
        self.cmd_vel_pub1 = self.create_publisher(Twist,"/turtle1/cmd_vel",1)
        self.cmd_vel_pub2 = self.create_publisher(Twist,"/turtle2/cmd_vel",1)
        self.timer_ = self.create_timer(0.005,self.send_velocity_command)
        self.get_logger().info("Drawing Circle in TurtleSim")
        self.cli = self.create_client(Spawn, '/spawn')
        self.pose1_subscriber = self.create_subscription(Pose,"/turtle1/pose",self.pose1_callback,10)
        self.pose2_subscriber = self.create_subscription(Pose,"/turtle2/pose",self.pose2_callback,10)
        self.once_runner = 1
        self.turtle1_runner = 1
        self.turtle2_runner = 0
        self.request_sender = 1
        self.req = Spawn.Request()

    def send_request(self):
        self.req.x = 5.544445
        self.req.y = 3.544445
        self.req.theta = 0.0
        self.cli.call_async(self.req)


    def send_velocity_command(self):
        if self.turtle1_runner == 1:
            print("running turtle 1")
            msg = Twist()
            msg.linear.x = 1.0
            msg.angular.z = 1.0
            self.cmd_vel_pub1.publish(msg)
        if self.turtle2_runner == 1:
            print("running turtle 2")
            msg2 = Twist()
            msg2.linear.x = 3.0
            msg2.angular.z = 1.0
            self.cmd_vel_pub2.publish(msg2)


    def pose1_callback(self,msg_in:Pose):
        if msg_in.theta >= -0.75 and msg_in.theta < 0.0:
            print(True)
            self.turtle1_runner = 0
            self.turtle2_runner = 1
            if self.request_sender == 1:
                self.send_request()
                self.request_sender = 0
            print(self.turtle2_runner)
            
    def pose2_callback(self,msg:Pose):

        if msg.theta >= -0.75 and msg.theta < 0:
            print(True)
            print("Done")
            self.turtle2_runner = 0

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
