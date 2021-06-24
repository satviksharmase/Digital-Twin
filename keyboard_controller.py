import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from pynput import keyboard
import getpass

class KeyboardController(Node):

    def on_release(self,key):
        msg = String()

        if key == keyboard.Key.up:
            print("Up")
            msg.data = "MOVEF:1000"
            self.publisher.publish(msg)
        if key == keyboard.Key.down:
            print("Down")
            msg.data = "MOVEB:1000"
            self.publisher.publish(msg)
        if key == keyboard.Key.shift:
            value1 = input("what is the speed with with you want to run the robo: ")
            print("Down")
            msg.data = "CONTB:%s" %value1
            self.publisher.publish(msg)
        if key == keyboard.Key.left:
            print("Left")
            msg.data = "TURNL:0100"
            self.publisher.publish(msg)
        if key == keyboard.Key.right:
            print("Right")
            msg.data = "TURNR:0100"
            self.publisher.publish(msg)
        if key == keyboard.Key.space:
            value = input("what is the speed with with you want to run the robo: ")
            print("speed up")
            msg.data = "CONTF:%s" %value
            self.publisher.publish(msg)
        if key == keyboard.KeyCode(char='+'):
        	print("Faster")
        	msg.data = "FASTO:600"
        	self.publisher.publish(msg)
        if key == keyboard.KeyCode(char='x'):
        	print("Fastest")
        	msg.data = "FASTT:600"
        	self.publisher.publish(msg)
        if key == keyboard.Key.esc:
            print("Stop")
            msg.data = "STOPR:0000"
            self.publisher.publish(msg)

    def __init__(self):
        super().__init__('KeyboardController')
        self.publisher = self.create_publisher(String, '/robot/control', 10)

        # Collect events until released
        with keyboard.Listener(
            on_release=self.on_release) as listener:
                listener.join()
        listener.start()


def main(args=None):
    rclpy.init(args=args)

    controller = KeyboardController()

    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()