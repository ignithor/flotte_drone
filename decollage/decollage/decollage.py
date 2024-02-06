import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from std_srvs.srv import Empty
from ls2n_interfaces.srv import DroneRequest


from ls2n_interfaces.msg import KeepAlive

qos_profile_sensor_data.depth = 1


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.get_logger().info("Minimal Publisher Node Created")

        drone: str = "Drone1"

#        self.joint_msg = JointTrajectory()
#        self.joint_msg.joint_names = ["x", "y", "z", "yaw"]

#        self.joint_point = JointTrajectoryPoint()
#        self.joint_point.positions = [2.0, 5.0, 5.0, 0.0]

#        self.joint_msg.points = [self.joint_point]

        self.keep_alive = KeepAlive()
        self.keep_alive.keep_alive = True
        self.keep_alive.feedback = False

#        self.traj_pub = self.create_publisher(
#            JointTrajectory, f"/{drone}/Trajectory", qos_profile_sensor_data
#        )

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

    def spin_motor(self) -> None:
        self.take_motor_client.call_async(Empty.Request())
        self.get_logger().info("Spin Motors Service Called")
        
    def take_off(self) -> None:
        self.take_off_client.call_async(Empty.Request())
        self.get_logger().info("Take Off Service Called")
        
    def start_experimentation(self, request, response):
        """
        Callback function of the service for start experimentation. All drones take-off when the service is called.
        :return: The response of the service.
        """
        self.get_logger().info('Start Experiment request made')
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.TAKE_OFF
        self.multi_control_client.call_async(request_out)

        return response

    def stop_experimentation(self, request, response):
        """
        Callback function of the service stop experimentation. All drones land when the service is called.
        :return response: The response of the service.
        """
        self.get_logger().info('Stop Experiment request made')
        msg = Bool()
        msg.data = False
        self.active = False
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.LAND
        self.multi_control_client.call_async(request_out)

        return response

    def spin_motors(
        self, request: DroneRequest, response: DroneRequest.Response
    ) -> DroneRequest.Response:
        self.get_logger().info("Spin Motors request received")

        if self.drone_status in [
            DroneStatus.IDLE,
            DroneStatus.PRE_ARMED,
        ]:
            self.network_request.request == NetworkRequest.Request.ACTIVATE
            self.network_request_client.call_async(self.network_request)

            self.drone_request.request == DroneRequest.Request.SPIN_MOTORS
            self.drone_request_client.call_async(self.drone_request)
            
            self.get_logger().info("Drone motors spinning")
            
        return response
    
    def spin_motors(
        self, request: DroneRequest, response: DroneRequest.Response
    ) -> DroneRequest.Response:


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
