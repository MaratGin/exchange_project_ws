import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool

class CoppeliaBridge(Node):
    def __init__(self):
        super().__init__('coppelia_bridge')
        self.sub = self.create_subscription(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.callback, 10)
        self.pub = self.create_publisher(Bool, '/kuka_joint_position_controller/joint_command', 10)

    def callback(self, msg):
        if not msg.points:
            return
        positions = msg.points[-1].positions  # Use final point
        out = Float32MultiArray()
        out.data = list(positions)
        self.pub.publish(out)
        self.get_logger().info(f"Sent joint positions: {out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = CoppeliaBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
