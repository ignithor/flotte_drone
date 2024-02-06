import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data

from ls2n_interfaces.msg import KeepAlive

qos_profile_sensor_data.depth = 1


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.get_logger().info("Minimal Publisher Node Created")
        self.declare_parameter("no_drone", "1")
        my_param = self.get_parameter("no_drone").get_parameter_value().string_value
        drone: str = "Drone" + my_param

        self.joint_msg = JointTrajectory()
        self.joint_msg.joint_names = ["x", "y", "z", "yaw"]
        x = 2.0
        y = 2.0
        z = 1.0
        yaw = 0.0
        if my_param == "1":
            y = y + 1.0
        if my_param == "2":
            x = x + 1.0
        if my_param == "3":
            x = x + 1.0
            y = y + 1.0
        self.joint_point = JointTrajectoryPoint()
        self.joint_point.positions = [x, y, z, yaw]

        self.joint_msg.points = [self.joint_point]

        self.keep_alive = KeepAlive()
        self.keep_alive.keep_alive = True
        self.keep_alive.feedback = False

        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{drone}/Trajectory", qos_profile_sensor_data
        )

        self.keepalive_pub = self.create_publisher(
            KeepAlive, f"/{drone}/KeepAlive", qos_profile_sensor_data
        )

        self.create_timer(0.05, self.publish_trajectory)
        self.create_timer(0.05, self.publish_keepalive)

    def publish_trajectory(self) -> None:
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.traj_pub.publish(self.joint_msg)
        self.get_logger().info("Published Trajectory")

    def publish_keepalive(self) -> None:
        self.keep_alive.stamp = self.get_clock().now().to_msg()
        self.keepalive_pub.publish(self.keep_alive)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
