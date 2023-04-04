import rclpy
from rclpy.node import Node

from tf2_ros import TransformException, TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from am_pp_interfaces.msg import FloatArrayAM
from am_pp_interfaces.action import PointcloudCapture
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose

class FrameListener(Node):

    def __init__(self):
        super().__init__('tf_reader')

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
          'target_frame', 'tool0').get_parameter_value().string_value
        self.tcp_name = self.declare_parameter('tcp_name','roscam_grip').get_parameter_value().string_value
        self.tcp_def = self.declare_parameter('tcp_def','0,-.10318,.09253').get_parameter_value().string_value

        self.tcp_def = list(self.tcp_def.split(","))
        self.tcp_def = [float(i) for i in self.tcp_def]
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create turtle2 velocity publisher
        self.pose_pub = self.create_publisher(Pose, '/current_pose', 1)
        self.table_pub = self.create_publisher(FloatArrayAM,'/tablecoords',1)
        self._action_client = ActionClient(self,PointcloudCapture,'pointcloudcapture')

        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def tcp_definition(self,msg):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'tool0'
        t.child_frame_id = self.tcp_name

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        ofs = self.tcp_def
        from_frame_rel = self.target_frame
        to_frame_rel = 'base'

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
        msg.position.x = t.transform.translation.x + ofs[0]
        msg.position.y = t.transform.translation.y - ofs[2]
        msg.position.z = t.transform.translation.z - ofs[1]

        msg.orientation.x = t.transform.rotation.x
        msg.orientation.y = t.transform.rotation.y
        msg.orientation.z = t.transform.rotation.z
        msg.orientation.w = t.transform.rotation.w

        self.pose_pub.publish(msg)
        self.send_pose(msg)
    
    def send_pose(self,pose):
        goal_msg = PointcloudCapture.Goal()
        goal_msg.pose = pose

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

        self._action_client.send_goal_async(goal_msg)

    def goal_response_callback(self,future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        print(result.table_coords)
        self.table_pub.publish(result.table_coords)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()