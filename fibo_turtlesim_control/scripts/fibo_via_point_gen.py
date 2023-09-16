#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,Point
from turtlesim.msg import Pose
from turtlesim_plus_interfaces.srv import GivePosition
from std_srvs.srv import Empty
from fibo_turtlesim_control.srv import SetGoal
import math
import yaml
import os

class GenViapoint(Node):
    def __init__(self):
        super().__init__('Scheduler')
        self.start_positions  = [[2.0, 6.0], [2.0, 1.0], [7.0, 6.0], [7.0, 1.0]]  # create start position of each letter ("F","I","B","O")
        self.store_via_point = [[] for _ in range(len(self.start_positions ))] # store via point of each letter
        self.walk_path = [[10, 5, 5], [8, 3, 3, 3, 3], [5, 10, 5],[8, 2, 8, 2]] # number of pizza for each path in each letter 
        self.walk_dir = [[0, 2, 2], [0, 3, 5, 3, 5], [2, 0, 2],  [0, 2, 4, 6]] # direction of walk (from 8 direction)
        self.tran_dis = 0.35 # translation length
        self.create_via_point() # call function to create viapoint
        
    def create_via_point(self)->None:
        TRAN_DIR = [ # store coorditiate of translation
            [0.0, 1.0],
            [0.707, 0.707],
            [1.0, 0.0], 
            [0.707, -0.707],
            [0.0, -1.0],
            [-0.707, -0.707,],
            [-1.0, 0.0],
            [-0.707, 0.707]
        ]
        # copy star_positions to store_via_point variable
        for index, pos in enumerate(self.start_positions ): 
            self.store_via_point[index].append(pos)

        # create viapoint
        for index_wp, path in enumerate(self.walk_path): 
            for index_p, pizza in enumerate(path):
                for index_pz in range(0, pizza):
                    if (index_pz != 0 or index_p != 0): # check if not the start_position 
                        dx, dy = TRAN_DIR[self.walk_dir[index_wp][index_p]] # get the coorditiate of translation
                        prev_x, prev_y = self.store_via_point[index_wp][-1] # get the lastest position
                        new_x = prev_x + self.tran_dis * dx # calculate the next position
                        new_y = prev_y + self.tran_dis * dy # calculate the next position
                        self.store_via_point[index_wp].append([new_x, new_y]) # append viapoint to store_via_point variable

        # offset path in "F" letter
        for index in range(15,20):
            self.store_via_point[0][index][0] = self.store_via_point[0][index][0] + (self.tran_dis * -5.0) 
            self.store_via_point[0][index][1] = self.store_via_point[0][index][1] + (self.tran_dis * -3.0) 

        # offset path in "I" letter
        for index in range(5,15):
            self.store_via_point[2][index][0] = self.store_via_point[2][index][0] + (self.tran_dis * -2.0)  

        # offset path in "I" letter
        for index in range(15,20):
            self.store_via_point[2][index][0] = self.store_via_point[2][index][0] + (self.tran_dis * -5.0) 

        # delcare path for store via_point_file
        output_dir = 'src/fibo_turtlesim_control/via_point'

        # iterate through self.store_via_point and generate YAML files for each index
        for index, via_point_data in enumerate(self.store_via_point):
            # create a filename based on the index
            filename = f"via_point_{index+1:02d}.yaml"  # Use 2-digit zero-padded index
            file_path = os.path.join(output_dir, filename)
            
            # write data to the YAML file
            self.data_to_yaml_file(file_path, "via_point", via_point_data)

    def data_to_yaml_file(self,path:str,namespace:str,array_data:list)->None:
        yaml_data = {namespace:array_data}
        with open(path, 'w') as file: # open path
            yaml.dump(yaml_data, file) # create yaml file

    def SetGoal(self, request, response):
        if request:
            request=False
            print(response.set_goal.x,response.set_goal.y)
        return response

    
def main(args=None):
    rclpy.init(args=args)
    node = GenViapoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()