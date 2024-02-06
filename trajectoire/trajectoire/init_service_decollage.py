from ls2n_interfaces.srv import DroneRequest
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Bool


class MCNode(Node):
    def __init__(self):
        super().__init__("multi_control_simulation")
        self.get_logger().info("Start multi control simulation")
        self.create_service(Empty, "StartExperiment", self.start_experimentation)
        self.create_service(Empty, "StopExperiment", self.stop_experimentation)
        self.create_service(Empty, "SpinMotors", self.spin_motors)
        self.multi_control_client = self.create_client(DroneRequest, "Request")

    def spin_motors(self, request, response):
        """
        Callback function of the service spin motors. All drones spin when the service is called.
        :return response: The response of the service.
        """
        self.get_logger().info("Spin motors request made")
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.SPIN_MOTORS
        self.multi_control_client.call_async(request_out)

        return response

    def start_experimentation(self, request, response):
        """
        Callback function of the service for start experimentation. All drones take-off when the service is called.
        :return: The response of the service.
        """
        self.get_logger().info("Start Experiment request made")
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.TAKE_OFF
        self.multi_control_client.call_async(request_out)

        return response

    def stop_experimentation(self, request, response):
        """
        Callback function of the service stop experimentation. All drones land when the service is called.
        :return response: The response of the service.
        """
        self.get_logger().info("Stop Experiment request made")
        msg = Bool()
        msg.data = False
        self.active = False
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.LAND
        self.multi_control_client.call_async(request_out)

        return response

    def activate_armada_control(self, request, response):
        self.get_logger().info("activate armada control request made")
        msg = Bool()
        msg.data = False
        self.active = False
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.ARMADA_CONTROL
        self.multi_control_client.call_async(request_out)

        return response


def main(args=None):
    rclpy.init(args=args)
    multi_control = MCNode()

    try:
        rclpy.spin(multi_control)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        multi_control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
