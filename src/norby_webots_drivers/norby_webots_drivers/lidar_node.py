import rclpy
from rclpy.node import Node
import sys
# Handles string messages
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from rclpy.qos import qos_profile_sensor_data


class LidarSubscriber(Node):
    """
    Create a subscriber node
    """

    def __init__(self):
        # Initiate the Node class's constructor and give it a name
        super().__init__('lidar_subscriber')

        # The node subscribes to messages of type std_msgs/String,
        # over a topic named: /addison
        # The callback function is called as soon as a message is received.
        # The maximum number of queued messages is 10.
        #qos_profile = qos_profile_sensor_data()
        # custom_qos = QoSProfile(
        #  depth=1,
        #  reliability=QoSReliabilityPolicy.BEST_EFFORT,
        #  durability=QoSDurabilityPolicy.VOLATILE)

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar',
            self.listener_callback,
            qos_profile_sensor_data

        )
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # Display a message on the console every time a message is received on the
        # addison topic
        self.get_logger().info('I heard: "%s"' % "")


def main():

    # Initialize the rclpy library
    rclpy.init(args=None)

    # Create a subscriber
    lidar_subscriber = LidarSubscriber()

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(lidar_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
