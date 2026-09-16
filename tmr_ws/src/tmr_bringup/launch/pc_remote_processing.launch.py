import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('tmr_bringup')
    params_file = os.path.join(pkg_share, 'config', 'vehicle_params.yaml')

    return LaunchDescription([
        # 1. Nodo de Visión OpenCV C++ (procesa la imagen comprimida y calcula errores)
        Node(
            package='tmr_vision',
            executable='lane_detector_node',
            name='lane_detector_node',
            parameters=[params_file],
            output='screen'
        ),

        # 2. Nodo de Control Lateral y Longitudinal (calcula steering y velocidad)
        Node(
            package='tmr_control',
            executable='lane_controller_node',
            name='lane_controller_node',
            parameters=[params_file],
            output='screen'
        )
    ])
