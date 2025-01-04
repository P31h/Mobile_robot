import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)  # Publish at 10 Hz
        self.get_logger().info('Twist Publisher Node has been started.')

    def publish_twist(self):
        twist = Twist()

        # Set linear velocity
        twist.linear.x = 0.5  # Forward
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        # Set angular velocity
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0  # Rotate counter-clockwise

        self.publisher_.publish(twist)
        

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly.')
    except Exception as e:
        node.get_logger().error(f'Error occurred: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()