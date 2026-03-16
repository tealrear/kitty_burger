from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='sdr_perception_cpp', executable='perception_node', name='perception'),
        Node(package='sdr_brain_system', executable='detect_human', name='detect_human'),
        Node(package='sdr_brain_system', executable='sdr_mission_controller', name='sdr_mission_controller'),
        Node(package='sdr_brain_system', executable='sdr_digit_reader', name='sdr_digit_reader'),
        Node(package='sdr_monitoring_station', executable='gui_node', name='gui_node'),
    ])