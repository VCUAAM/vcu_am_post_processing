import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf_reader')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'base').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(Pose, '/current_pose', 1)

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base'
        to_frame_rel = self.target_frame

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        msg = Pose()
        msg.position.x = t.transform.translation.x
        msg.position.y = t.transform.translation.y
        msg.position.z = t.transform.translation.z

        msg.orientation.x = t.transform.rotation.x
        msg.orientation.y = t.transform.rotation.y
        msg.orientation.z = t.transform.rotation.z
        msg.orientation.w = t.transform.rotation.w

        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass