import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading

class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1]
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1]
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)


class Robot(Node):
    def listener_callback(self, msg):
        print("exciting")
        command = msg.data #the actual command string
        command +="\n" #add the newline character
        self.get_logger().info('Sending data to serial port: "%s"' % msg.data)
        ser = serial.Serial('/dev/ttyACM0')  # open serial port
        print(ser.name)         # check which port was really used
        ser.write(command.encode())
        ser.close()             # close port

    
    def __init__(self):
        super().__init__('robot_bridge')

        
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
        self.publisher_left = self.create_publisher(String, '/robot/left', 10)
        self.publisher_front = self.create_publisher(String, '/robot/front', 10)
        self.publisher_right = self.create_publisher(String, '/robot/right', 10)

        
        ser = serial.Serial(
            port='/dev/ttyACM0',\
            baudrate=9600,\
            parity=serial.PARITY_NONE,\
            stopbits=serial.STOPBITS_ONE,\
            bytesize=serial.EIGHTBITS,\
            timeout=0)

        rl = ReadLine(ser)

        while(True):
            line = rl.readline().decode()
            if(len(line)>10):
                clean = line[1:] #removes the [                                       
                clean2 = clean[:-2] #removes the ]\n                                  
                readings = clean2.split(',')
                msg = String()

                msg.data = readings[0]
                self.publisher_left.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)

                msg.data = readings[1]
                self.publisher_front.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)

                msg.data = readings[2]
                self.publisher_right.publish(msg)
                self.get_logger().info('Publishing: "%s"' % msg.data)


        

def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    rclpy.spin(robot)
    
    robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()