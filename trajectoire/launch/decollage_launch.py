from launch import LaunchDescription
from launch_ros.actions import Node
from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.node(
        "ls2n_drone_command_center",
        "multi_control",
        parameters={"drones_to_control": ["Drone1"]},
    )
    return sl.launch_description()
