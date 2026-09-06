import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class CommandMapperNode(Node):
    def __init__(self):
        super().__init__('command_mapper')
        self.subscription = self.create_subscription(String, '/voice_commands', self.listener_callback, 10)
        self.scale_subscription = self.create_subscription(Float32, '/voice_control/speed_scale', self.scale_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed_scale = 1.0
        self.get_logger().info("CommandMapperNode ready.")

    def scale_callback(self, msg):
        self.speed_scale = max(0.25, min(1.5, float(msg.data)))
        self.get_logger().info(f"Speed scale updated: {self.speed_scale:.2f}")

    def listener_callback(self, msg):
        cmd = msg.data.lower()
        twist = Twist()

        if self.matches(cmd, ("forward", "go forward", "move forward", "ahead", "前进", "往前", "向前")):
            twist.linear.x = 0.12 * self.speed_scale
        elif self.matches(cmd, ("backward", "go back", "move back", "back", "后退", "往后", "向后", "倒退")):
            twist.linear.x = -0.10 * self.speed_scale
        elif self.matches(cmd, ("left", "turn left", "go left", "左转", "向左", "往左")):
            twist.angular.z = 0.35 * self.speed_scale
        elif self.matches(cmd, ("right", "turn right", "go right", "右转", "向右", "往右")):
            twist.angular.z = -0.35 * self.speed_scale
        elif self.matches(cmd, ("stop", "halt", "freeze", "pause", "停车", "停止", "停下", "别动")):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
        else:
            self.get_logger().info(f"Ignored: {cmd}")
            return

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Sent movement command for: {cmd}")

    def matches(self, command, aliases):
        return any(alias in command for alias in aliases)

def main(args=None):
    rclpy.init(args=args)
    node = CommandMapperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
