import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Twist

#for calculating position
import math
from math import sin, cos, pi

#for threading
import threading

import time

class RobotDigitalTwin(Node):
    #the location of the robot, x, y and rotation
    x=0.0
    y=0.0
    z=0.5
    angle = 0.0 #in degrees
    th=0.0 #angle in radians
        
    def listener_callback(self, msg):
        command = msg.data[0:5] #the actual command string
        value = int(msg.data[6:11]) #the data
        print(command)
        print(value)

        #assume 1000 is 10cm
        #assume 100 is 10 degrees
        #The following handles rotation and movement.
        
        if(command == "MOVEF"):
            if(value == 1000): 
                print("robot moving forward")
                vx = 0.1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th      
        elif(command == "FASTO"):
            if(value == 600):
                print("robot moving fast")
                vx = 1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
                
                
        elif(command == "FASTO"):
            if(value == 600):
                print("robot moving fast")
                vx = 1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
              
        elif(command == "FASTT"):
            if(value == 600):
                print("robot moving fastest")
                vx = 1.5 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th        
              
        elif(command == "FASTT"):
            if(value == 600):
                print("robot moving fastest")
                vx = 1.5 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x += delta_x
                self.y += delta_y
                self.th += delta_th
             
        elif(command == "MOVEB"):
            if(value == 1000): 
                print("robot moving backwards")
                vx = 0.1 #velocity of x, how much we move
                # compute odometry
                delta_x = vx * cos(self.th)
                delta_y = vx * sin(self.th)
                delta_th = 0 #no change in angle
                #add the changes to the values
                self.x -= delta_x
                self.y -= delta_y
                self.th -= delta_th

        #Obey the right-hand law. Orientation is counter-clockwise.
        elif(command == "TURNR"):
            print("robot turning left")
            if(value == 100):
                self.angle = (self.angle - 10) #if the result is +ve
                if (self.angle < 0): self.angle = 360 - abs(self.angle) #if the result is -ve
                self.th = math.radians(self.angle) #convert to radians
                
        elif(command == "TURNL"):
            print("robot turning right")
            if(value == 100):
                self.angle = ((10 + self.angle) % 360 ) #add to the angle, keeping mod 360
                self.th = math.radians(self.angle) #convert to radians

        print(self.x)        
        print(self.y)
        print(self.z)
        print(self.angle)
        print(self.th)
    
    def __init__(self):
        super().__init__('robot_digital_twin')

        self.subscription = self.create_subscription(
            String,
            '/robot/control',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # thread for publishing messages from the serial port
        self.publish_thread = threading.Thread(target=self._publish_thread)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        
            
    def _publish_thread(self):
        #publish the pose regularly
        while(True):
            self.publisher = self.create_publisher(Twist, '/robot/pose', 10)
            msg = Twist()
            msg.linear.x = self.x
            msg.linear.y = self.y
            msg.linear.z = self.z #for height
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = self.th

            self.publisher.publish(msg)
            self.get_logger().info("Publishing to /robot/pose")
            time.sleep(0.2) #1/5 of a second


def main(args=None):
    rclpy.init(args=args)

    robot = RobotDigitalTwin()
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()