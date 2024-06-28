from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    nuber_node_name = "my_number_publisher"
    
    number_node = LifecycleNode(
        package="lifecycle_py",
        executable="number_publisher",
        name=nuber_node_name,
        namespace=""
    )
    
    node_manager= Node(
        package="lifecycle_py",
        executable="lifecycle_node_manager",
        name="my_node_manager",
        parameters=[
            {"managed_node_name": nuber_node_name}
        ]
    )
    
    ld.add_action(number_node)
    ld.add_action(node_manager)
    
    return ld