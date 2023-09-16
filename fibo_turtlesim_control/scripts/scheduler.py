#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from fibo_turtlesim_control.srv import SetGoal
from ament_index_python.packages import get_package_share_directory
import math
from std_msgs.msg import Bool

import yaml
import os

class Scheduler(Node):
    def __init__(self):
        super().__init__('Scheduler')
        self.declare_parameter('index',0)
        self.index = self.get_parameter('index').value 
        self.create_service(SetGoal, 'fibo_turtle_control/set_goal', self.SetGoal) # create service to set goal for turtle
        self.yaml_data = []
        self.data_index = 0
        self.read_yaml_data() # call read_yaml_data 

    def SetGoal(self, request, response): # via_point service for senting via_point to turtle
        response.set_goal.x = self.yaml_data[self.data_index][0]
        response.set_goal.y = self.yaml_data[self.data_index][1]
        self.data_index+=1
        return response
    
    def read_yaml_data(self): # read ymal_file
        yaml_path = 'src/fibo_turtlesim_control/via_point/via_point_0'+str(self.index+1)+'.yaml'
       
        with open(yaml_path, 'r') as yaml_file: # read file
            self.yaml_data = yaml.load(yaml_file,Loader=yaml.SafeLoader)
            self.yaml_data = self.yaml_data['via_point']





def main(args=None):
    rclpy.init(args=args)
    node = Scheduler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()