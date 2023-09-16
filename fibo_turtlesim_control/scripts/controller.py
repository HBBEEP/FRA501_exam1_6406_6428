#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose 
from turtlesim_plus_interfaces.srv import GivePosition
import math
from fibo_turtlesim_control.srv import SetGoal
from std_srvs.srv import Empty
from std_msgs.msg import Bool

class Controller(Node):
    def __init__(self):
        super().__init__('Controller')
        self.cli = self.create_client(SetGoal, 'fibo_turtle_control/set_goal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.via_point_data = [] 
        self.req = SetGoal.Request()
        self.req.result = False
        for _ in range(20):
            self.via_point_data.append(self.call_setGoal()) # append viapoint to via_point_data

        self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(0.01, self.timer_callback) # timer
        self.create_subscription(Pose, "/pose", self.turtle_pose_callback, 10) # create sub scription for receiving pose from self.turtle_pose_callback 
        self.create_subscription(Bool,"/get_out",self.get_out,10) # create sub scription for receiving get_out status
        self.spawn_pizza_client = self.create_client(GivePosition, "/spawn_pizza") # create client for spawning pizza
        self.send_standby = self.create_publisher(Bool,"/standby",10) # create publisher for senting standby status
        
        self.turtle_current_pose = [0.0, 0.0, 0.0]
        self.turtle_target_pose = [0.0, 0.0]
        self.kp_dis = 5.0 # p-term of cmd_vel.linear.x 
        self.kp_ori = 20.0 # p-term of cmd_vel.angular.z 
        self.isEnableController = False
        self.via_point_index = 0
        self.target_pose()

    def get_out(self, status):
        if (status.data == True):
            self.turtle_target_pose = [10.0,10.0]
            self.isEnableController = True

    def call_setGoal(self):
        self.req.result = False
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future,timeout_sec=1.0)
        return self.future.result()

    def target_pose(self): # assign target 
        if (self.via_point_index <= 19):
            self.turtle_target_pose = [self.via_point_data[self.via_point_index].set_goal.x, self.via_point_data[self.via_point_index].set_goal.y]
            self.isEnableController = True
            self.via_point_index+=1
        else:
            message = Bool()
            message.data = True 
            self.send_standby.publish(message)

    def turtle_pose_callback(self, msg):
        self.turtle_current_pose = [msg.x, msg.y, msg.theta]

    def cmd_vel(self, vx, w):
        cmd_vel = Twist()
        cmd_vel.linear.x = vx
        cmd_vel.angular.z = w
        self.pub_cmd_vel.publish(cmd_vel) # make the turtle move

    def spawn_pizza(self, position): # spawn pizza 
        position_request = GivePosition.Request()
        position_request.x = position[0]
        position_request.y = position[1]
        self.spawn_pizza_client.call_async(position_request)

    def timer_callback(self):
        if self.isEnableController: # check status of controller is ready to go to via point
            self.controller()

    def controller(self): 
        dx = self.turtle_target_pose[0] - self.turtle_current_pose[0] # find diff position : x
        dy = self.turtle_target_pose[1] - self.turtle_current_pose[1] # find diff position : y
        alpha = math.atan2(dy, dx)
        e_dis = math.hypot(dx, dy)
        e_ori = alpha - self.turtle_current_pose[2]
        e_ori = math.atan2(math.sin(e_ori), math.cos(e_ori))
        u_dis = e_dis * self.kp_dis
        u_ori = e_ori * self.kp_ori
        if e_dis <= 0.05: # check diff error
            self.isEnableController = False
            self.cmd_vel(0.0, 0.0) # set cmd_vel.linear.x cmd_vel.angular.z to zero for stop moving
            self.spawn_pizza(self.turtle_target_pose) # spwaning pizza
            self.target_pose()
    
        else:
            self.cmd_vel(u_dis, u_ori)

def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
