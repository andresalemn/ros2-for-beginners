#!/usr/bin/env python3  

import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test")             #Give name to the node
        self.counter_ = 0
        self.get_logger().info("Hello ROS2")    #Logs a message with info
        self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)   #Initialize the node and ROS2 communication
    node = MyNode()         #Give name to the node
    rclpy.spin(node)        #Pause, give spin (useful with callbacks)
    rclpy.shutdown()        #Shutdown the ROS2 communication

#Basic python program structure
if __name__=="__main__":
    main()

""" Notes
- The node is not the executable nor the file
- The name of the node is not the name of the file
- The node is created inside the file

shebang -->(#+!) 
El shebang es una línea que se encuentra al principio de un archivo de script 
y que especifica qué programa debe llamarse para ejecutar el script.
Execute with a Python interpreter, using the env program search path to find it
"""