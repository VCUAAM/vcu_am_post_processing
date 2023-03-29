import time
import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.goalPose = Pose
        # Create turtle2 velocity publisher
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 1)
        #self.subscription = self.create_subscription(Pose,'test_pose',self.listener_callback,10)
        
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        goalpos = self.goalPose
        #try:
            
        msg = Pose()
        """
        msg.position.x = goalpos.position.x
        msg.position.y = goalpos.position.y
        msg.position.z = goalpos.position.z

        msg.orientation.x = goalpos.orientation.x
        msg.orientation.y = goalpos.orientation.y
        msg.orientation.z = goalpos.orientation.z
        msg.orientation.w = goalpos.orientation.w
        """
        msg.position.x = 0.0
        
        msg.position.y = .225
        msg.position.z = 1.079

        msg.orientation.x = -0.7041968077872236
        msg.orientation.y = 0.002888251404049071
        msg.orientation.z = -0.0028862188011961826
        msg.orientation.w = 0.7099930870417984
        
        pos = PoseStamped()
        head = Header()
        pos.header = head
        pos.pose = msg
        self.publisher.publish(pos)
        print(pos)
        #except:
            #time.sleep(1)

    #def listener_callback(self, msg):
        #self.goalPose = msg

def main(args=None):
    rclpy.init(args=args)
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()