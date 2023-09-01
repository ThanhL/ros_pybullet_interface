import pybullet as pb

from ros_pybullet_interface.pybullet_instance import PybulletInstance
# from ros_node import RosNode

import rclpy
from rclpy.node import Node


class RPBINode(Node):
    def __init__(self, *args, **kwargs):
        print("[+] Initializing node!")
        # Initialize node
        node_name = args[0]
        super().__init__(node_name)
        
        # # Get configuration
        # self.config = self.get_parameter('~config')
        # print(self.config)
            
        # # Connect to pybullet
        # self.pybullet_instance = PybulletInstance(pb, self)


def main(args=None):
    # Init
    rclpy.init(args=args)
    rpbi_node = RPBINode('rbpi_node')
    
    # Spin
    rclpy.spin(rpbi_node)

    # Destroy 
    rpbi_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()