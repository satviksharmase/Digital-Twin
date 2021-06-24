import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import time
from geometry_msgs.msg import Twist
import math

class RobotVisualiser(Node):

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Converts euler roll, pitch, yaw to quaternion (w in last place)
        quat = [x, y, z, w]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = sy * cp * cr - cy * sp * sr
        q[2] = cy * cp * sr - sy * sp * cr
        q[3] = sy * cp * sr + cy * sp * cr

        return q

    def listener_callback(self, msg):

        #now move the object
        client = self.create_client(SetEntityState, "/world/set_entity_state")
        self.get_logger().info("Connecting to `/world/set_entity_state` service...")

        # Set data for request
        request = SetEntityState.Request()
        entitystate = EntityState()
        entitystate.name= "wall"
        request.state = entitystate
        request.state.pose.position.x = msg.linear.x
        request.state.pose.position.y = msg.linear.y
        request.state.pose.position.z = msg.linear.z
        q = self.quaternion_from_euler(msg.angular.x,msg.angular.y,msg.angular.z)
        request.state.pose.orientation.x = q[0]
        request.state.pose.orientation.y = q[1]
        request.state.pose.orientation.z = q[2]
        request.state.pose.orientation.w = q[3]
        print(request)
        self.get_logger().info("Sending service request to `/world/set_entity_state`")
        future = client.call_async(request)

    def __init__(self):
        super().__init__('wall_visualiser')

        #first spawn
        self.get_logger().info(
            'Creating Service client to connect to `/spawn_entity`')
        client = self.create_client(SpawnEntity, "/spawn_entity")

        self.get_logger().info("Connecting to `/spawn_entity` service...")
        if not client.service_is_ready():
            client.wait_for_service()
            self.get_logger().info("...connected!")

        # Set data for request
        request = SpawnEntity.Request()
        request.name = 'wall'
        request.xml = "<?xml version='1.0'?><sdf version='1.4'><model name='my_model'><pose>5 0.5 0 0 0</pose><static>true</static><link name='link'><collision name='collision'><geometry><box><size>0.2 2.5 3</size></box></geometry></collision><visual name='visual'><geometry><box><size>0.2 2.5 3</size></box></geometry></visual></link></model></sdf>"        
        request.robot_namespace = "demo"

        self.get_logger().info("Sending service request to `/spawn_entity`")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            print('response: %r' % future.result())
        else:
            raise RuntimeError(
                'exception while calling service: %r' % future.exception())

        #subscribe to the post channel.
        self.subscription = self.create_subscription(Twist, '/robot/wall', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

def main(args=None):
    rclpy.init(args=args)

    visualiser = RobotVisualiser()
    rclpy.spin(visualiser)

    visualiser.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()