#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import tty
import sys
import termios




class PropellerPublisher(Node):
    def __init__(self):
        super().__init__('propeller_publisher')
        self.publisher0_ = self.create_publisher(Float64, '/Orca/propeller0', 10)
        self.publisher1_ = self.create_publisher(Float64, '/Orca/propeller1', 10)
        self.publisher2_ = self.create_publisher(Float64, '/Orca/propeller2', 10)
        self.publisher3_ = self.create_publisher(Float64, '/Orca/propeller3', 10)
        self.publisher4_ = self.create_publisher(Float64, '/Orca/propeller4', 10)
        self.publisher5_ = self.create_publisher(Float64, '/Orca/propeller5', 10)
        self.publisher6_ = self.create_publisher(Float64, '/Orca/propeller6', 10)
        self.publisher7_ = self.create_publisher(Float64, '/Orca/propeller7', 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        speed = -10.0                            #propeller thrust speed
        msg1 = Float64()
        msg1.data = float(speed)
        msg0 = Float64()
        msg0.data = float(0)
        # Read keyboard input
        key = input("Enter propeller value (float): ")
        if key == 'w':
            print("Forward thuster active !!")
            self.publisher4_.publish(msg1)
            self.publisher7_.publish(msg1)
            self.publisher5_.publish(msg0)
            self.publisher6_.publish(msg0)
        elif key == 's':
            print("Reverse thuster active !!")
            msg = Float64()
            msg.data = float(speed)
            self.publisher5_.publish(msg1)
            self.publisher6_.publish(msg1)
            self.publisher4_.publish(msg0)
            self.publisher7_.publish(msg0)
        elif key == 'd':
            print("Strafing right !!")
            msg = Float64()
            msg.data = float(speed)
            self.publisher4_.publish(msg1)
            self.publisher6_.publish(msg1)
            self.publisher5_.publish(msg0)
            self.publisher7_.publish(msg0)

        elif key == 'a':
            print("Strafing left !!")
            msg = Float64()
            msg.data = float(speed)
            self.publisher4_.publish(msg0)
            self.publisher6_.publish(msg0)
            self.publisher5_.publish(msg1)
            self.publisher7_.publish(msg1)
        
        elif key == 'e':
            print("Steering right !!")
            msg = Float64()
            msg.data = float(speed)
            self.publisher4_.publish(msg0)
            self.publisher6_.publish(msg1)
            self.publisher5_.publish(msg0)
            self.publisher7_.publish(msg1)
        
        elif key == 'q':
            print("Steering left !!")
            msg = Float64()
            msg.data = float(speed)
            self.publisher4_.publish(msg1)
            self.publisher6_.publish(msg0)
            self.publisher5_.publish(msg1)
            self.publisher7_.publish(msg0)
        
        else:
            print("Propellers off !!")
            self.publisher4_.publish(msg0)
            self.publisher6_.publish(msg0)
            self.publisher5_.publish(msg0)
            self.publisher7_.publish(msg0)

            

        # Publish the input value to the topics
        
        

def main(args=None):
    rclpy.init(args=args)
    propeller_publisher = PropellerPublisher()
    rclpy.spin(propeller_publisher)
    propeller_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()