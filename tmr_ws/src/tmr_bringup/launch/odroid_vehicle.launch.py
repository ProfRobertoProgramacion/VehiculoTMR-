import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tmr_bringup')
    params_file = os.path.join(pkg_share, 'config', 'vehicle_params.yaml')

    return LaunchDescription([
        # 1. Nodo de captura de cámara USB (publica /camera/image_raw/compressed)
        Node(
            package='tmr_camera',
            executable='usb_camera_node',
            name='usb_camera_node',
            parameters=[params_file],
            output='screen'
        ),

        # 2. Nodo de comunicación con Arduino (recibe /cmd_vel y escribe en /dev/ttyACM0)
        Node(
            package='tmr_hardware',
            executable='arduino_bridge_node',
            name='arduino_bridge_node',
            parameters=[params_file],
            output='screen'
        )
    ])
