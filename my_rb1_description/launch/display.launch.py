import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get package directory
    package_description = "my_rb1_description"
    package_directory = get_package_share_directory(package_description)

    # Load URDF File #
    urdf_file = 'my_rb1_robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print(robot_desc_path)
    print("URDF Loaded !")

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path])}]
    )

    # Load RViz config file
    rviz_config_file = "config.rviz"
    rviz_config_path = os.path.join(package_directory, "rviz", rviz_config_file)
    print(rviz_config_path)
    print("RViz Config Loaded !")

    # Rviz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_path],
    )

    # The joint state control GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        emulate_tty=True,
    )

    # Return the LaunchDescription object
    return LaunchDescription(
        [
            robot_state_publisher,
            rviz2,
            joint_state_publisher_gui,
        ]
    )