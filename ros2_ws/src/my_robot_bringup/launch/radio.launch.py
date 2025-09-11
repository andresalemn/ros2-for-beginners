from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    giskard_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_giskard",
        parameters=[
            {"robot_name": "Giskard Reventlov"}
        ]
    )

    bb8_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_bb8",
        parameters=[
            {"robot_name": "BB-8"}
        ]
    )

    daneel_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_daneel",
        parameters=[
            {"robot_name": "Daneel Olivaw"}
        ]
    )

    jander_node = Node(
        package="my_py_pkg",
        executable="robot_news_station",
        name="robot_news_station_jander",
        parameters=[
            {"robot_name": "Jander Panell"}
        ]
    )

    c3po_node = Node(
        package="my_cpp_pkg",
        executable="robot_news_station",
        name="robot_news_station_c3pos",
        parameters=[
            {"robot_name": "CRPO"}
        ]
    )

    smartphone = Node(
        package="my_cpp_pkg",
        executable="smartphone",
        name="smartphone",
    )

    ld.add_action(giskard_node)
    ld.add_action(bb8_node)
    ld.add_action(daneel_node)
    ld.add_action(jander_node)
    ld.add_action(c3po_node)
    ld.add_action(smartphone)

    return ld

