import os 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import yaml

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ros_pybullet_interface'),
        'config',
        'config.yaml'
        )

    print(f"[+] config: {config}")
    with open(config, 'r') as f:
        params = yaml.safe_load(f)

    print(params)

    node = Node(
        package='ros_pybullet_interface',
        executable='rpbi_node',
        name='rpbi_node',
        output="screen",
        parameters=[
            config
        ]
    )
    
    ld = LaunchDescription()
    ld.add_action(node)
    return ld