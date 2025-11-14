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
        self.publisher_ = self.create_publisher(Bool, 'isLight', 10)  # Create a publisher for bool type messages on the topic mainsensortopic
        self.declare_parameter('msg_frequency', 10)                        # Instantiate parameter, set default value to frequency 10
        self.declare_parameter('ir_pin_#', IR1_INPUT_PIN)            # Instantiate parameter, set default value to pin 1
        timer_period = 0.5                                                  # Define the timer period in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)   # Create a timer that calls 'timer_callback' every 0.5 seconds

    def timer_callback(self):
        my_param = self.get_parameter('msg_frequency').get_parameter_value().integer_value
  

        my_param2 = self.get_parameter('ir_pin_#').get_parameter_value().integer_value

        msg = Bool()

        self.get_logger().info("Reading IR1")
        ir1Value = get_ir_state(my_param2)
        if(ir1Value == DARK):
            self.get_logger().info("IR1 is Dark")
            msg.data = False
            self.publisher_.publish(msg)    
        elif(ir1Value == LIGHT):
            self.get_logger().info("IR1 is LIGHT")
            msg.data = True
            self.publisher_.publish(msg)    
        elif(ir1Value == INVALID):
            self.get_logger().info("Invalid!")
        time.sleep(2)

                                # Publish the message to the topic
        #self.get_logger().info('Publishing: "%s"' % msg.data)   # Log the published message for debugging


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