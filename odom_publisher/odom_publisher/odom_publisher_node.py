import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Point, Twist
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs


class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.timer = self.create_timer(0.1, self.publish_odom)
        self.latest_twist = Twist()

    def cmd_vel_callback(self, msg):
        self.latest_twist = msg

    def publish_odom(self):
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'chassis', rclpy.time.Time())
            
            odom = Odometry()
            odom.header.stamp = transform.header.stamp
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'chassis'

            # Populate pose
            odom.pose.pose.position = Point(
                x=transform.transform.translation.x,
                y=transform.transform.translation.y,
                z=transform.transform.translation.z
            )
            odom.pose.pose.orientation = Quaternion(
                x=transform.transform.rotation.x,
                y=transform.transform.rotation.y,
                z=transform.transform.rotation.z,
                w=transform.transform.rotation.w
            )

            # Populate twist from the latest cmd_vel message
            odom.twist.twist = self.latest_twist

            # Set covariance as identity matrix
            odom.pose.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]

            # Set twist covariance as identity matrix
            odom.twist.covariance = [
                1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 1.0
            ]

            self.odom_publisher.publish(odom)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'Exception occurred: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdometryPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
