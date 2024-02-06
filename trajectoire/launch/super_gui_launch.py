from launch import LaunchDescription
from launch_ros.actions import Node
from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.node("ls2n_drone_ui", "drone_ui")
    sl.node("ls2n_drone_ui", "drone_ui")
    sl.node("ls2n_drone_ui", "drone_ui")
    sl.node("ls2n_drone_ui", "drone_ui")
    return sl.launch_description()
