#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class TurtleStandby(Node):
    def __init__(self):
        super().__init__('Turtle_standby')
        self.init_time = 0 
        self.start_init = False
        self.count_finish = 0
        self.create_subscription(Bool,"/standby",self.standby,10) # create subscription for checking turtle movement that have finished drawing the letter
        self.send_get_out = self.create_publisher(Bool,"/get_out",10)
        self.create_timer(0.01, self.timer_callback)
        
    def timer_callback(self): # timer 
        if (self.start_init == True and self.count_finish == 4):
            self.init_time += 0.01
        if (self.init_time >= 5.0):
            self.start_init = False
            message = Bool()
            message.data = True
            self.send_get_out.publish(message)
    
    def standby(self, status): # count the number of turtles that have finished drawing the letter
        self.count_finish += 1
        self.start_init = status.data

def main(args=None):
    rclpy.init(args=args)
    node = TurtleStandby()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()