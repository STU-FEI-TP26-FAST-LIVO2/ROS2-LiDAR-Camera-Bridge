import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share   = get_package_share_directory('lidar_stamp_bridge')
    params      = os.path.join(pkg_share, 'config', 'lidar_stamp_bridge.yaml')
    cyclone_xml = os.path.join(pkg_share, 'config', 'cyclonedds.xml')

    return LaunchDescription([
        # --- DDS tuning (musi sediet s Hesai start.py) --------------------
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI',     'file://' + cyclone_xml),

        # --- Bridge node --------------------------------------------------
        Node(
            package='lidar_stamp_bridge',
            executable='lidar_stamp_bridge_node',
            name='lidar_stamp_bridge_node',
            output='screen',
            parameters=[params],
        ),
    ])
