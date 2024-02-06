from launch import LaunchDescription
from launch_ros.actions import Node
from simple_launch import SimpleLauncher


def generate_launch_description():
    sl = SimpleLauncher()
    sl.node("trajectoire", "trajectoire", parameters={"no_drone": "1"})
    sl.node("trajectoire", "trajectoire", parameters={"no_drone": "2"})
    sl.node("trajectoire", "trajectoire", parameters={"no_drone": "3"})
    sl.node("trajectoire", "trajectoire", parameters={"no_drone": "4"})
    return sl.launch_description()
