import rclpy                    # import the ROS Client Library for Python (RCLPY)
from rclpy.node import Node     # from RCLPY, import the Node Class used to create ROS 2 nodes
from std_msgs.msg import Int32 # from standard messages, import the int32 message
from std_msgs.msg import Bool

import os
include_dir = os.path.dirname(os.path.realpath(__file__)) + "/../../../../../../src/include/"
import sys
sys.path.append(include_dir)
from hat_library import *  #Import hatlibrary

class MinimalPublisher(Node):   # Create a new class called MinimalPublisher that inherits variables & functions from Node

    def __init__(self):
        super().__init__('minimal_publisher')                               # Initialize the Node with the name 'minimal_publisher'
        self.publisher_ = self.create_publisher(Bool, 'isLight', 10)  # Create a publisher for bool type messages on the topic isLight
        self.declare_parameter('msg_frequency', 10)                        # Instantiate parameter, set default value to frequency 10
        self.declare_parameter('sensor_#', 1)            # Instantiate parameter, set default value to pin 1
        timer_period = 1.0/self.get_parameter('msg_frequency').get_parameter_value().integer_value  # Define the timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)   # Create a timer that calls 'timer_callback' every 0.5 seconds

    def timer_callback(self):
        
        my_param2 = self.get_parameter('sensor_#').get_parameter_value().integer_value
        msg = Bool()
       
        if my_param2 == 1:
            pinNum = IR1_INPUT_PIN
        else:
            pinNum = IR2_INPUT_PIN

        self.get_logger().info("Reading Sensor")
        irValue = get_ir_state(pinNum)
        if(irValue == DARK):
            self.get_logger().info("Sensor is Dark")
            msg.data = False
            self.publisher_.publish(msg)    
        elif(irValue == LIGHT):
            self.get_logger().info("Sensor is LIGHT")
            msg.data = True
            self.publisher_.publish(msg)    
        else:
            self.get_logger().info("Invalid!")
        



def main(args=None):
    print ("Beginning to talk...")          # Print a starting message
    rclpy.init(args=args)                   # Initialize the ROS 2 Python client library

    minimal_publisher = MinimalPublisher()  # Create an instance of the MinimalPublisher class

    try:
        rclpy.spin(minimal_publisher)       # Keep the node active and processing callbacks until interrupted

    except KeyboardInterrupt:   # Handle a keyboard interrupt (Ctrl+C)
        print("\n")             # Print a newline for better format
        print("Stopping...")    # Print a stopping message
 
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        minimal_publisher.destroy_node()
        if rclpy.ok():                      # Check if the rclpy library is still running
            rclpy.shutdown()                # Shut down the ROS 2 client library, cleanly terminating the node



if __name__ == '__main__':
    main()                  # Call the main function to execute the code when the script is run