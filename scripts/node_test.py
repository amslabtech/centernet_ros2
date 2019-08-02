import rclpy
from std_msgs.msg import String

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('node')
    publisher = node.create_publisher(String, 'topic')

    msg = String()
    i = 0

    def timer_callback():
        nonlocal i
        msg.data = 'Hello World! ^^: %d' % i
        i += 1
        node.get_logger().info('Publishing: "%s"' % msg.data)
        publisher.publish(msg)

    timer_period = 0.5
    timer = node.create_timer(timer_period, timer_callback)

    rclpy.spin(node)

    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
