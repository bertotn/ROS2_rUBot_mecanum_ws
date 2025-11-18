from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    my_robot_control_node = Node(
        package="my_robot_control",
        executable="my_robot_control_lab_exec",
        name="robot_control",
        parameters=[
            {"vx": 0.3},
            {"vy": 0.0},
            {"w": 0.0},
            {"td": 2.0}
        ]
    )
    ld.add_action(my_robot_control_node)
    return ld