from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='three_aircraft_control',
            #namespace='demo1',
            executable='three_aircraft_control_demo',
            name='Vehicle1'
            ),

        Node(
            package='three_aircraft_control',
            #namespace='demo1',
            executable='three_aircraft_control_demo',
            name='Vehicle2'
            ),

        Node(
            package='three_aircraft_control',
            #namespace='demo1',
            executable='three_aircraft_control_demo',
            name='Vehicle3'
            )
        ])
