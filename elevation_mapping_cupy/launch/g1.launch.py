import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    package_name = 'elevation_mapping_cupy'
    share_dir = get_package_share_directory(package_name)
    
    core_param_path = os.path.join(share_dir, 'config', 'setups', 'g1', 'g1_parameters.yaml')
    sensor_param_path = os.path.join(share_dir, 'config', 'setups', 'g1', 'g1_sensor_parameter.yaml')
    
    # Add verification
    if not os.path.exists(core_param_path):
        raise FileNotFoundError(f"Unitree G1 core param file not found: {core_param_path}")
    if not os.path.exists(sensor_param_path):
        raise FileNotFoundError(f"Unitree G1 sensor param file not found: {sensor_param_path}")

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            share_dir, 'rviz', 'turtle_sim_laser.rviz'
        ]),
        description='Path to the RViz config file'
    )
    rviz_config = LaunchConfiguration('rviz_config')

    use_python_node_arg = DeclareLaunchArgument(
        'use_python_node',
        default_value='false',
        description='Use the Python node if true'
    )
    use_python_node = LaunchConfiguration('use_python_node')

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    elevation_mapping_node_py = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node.py',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            ParameterFile(core_param_path, allow_substs=True), 
            sensor_param_path,
            {'use_sim_time': use_sim_time}
        ],
        condition=IfCondition(use_python_node)
        # condition=IfCondition(PythonExpression(use_python_node))
    )

    elevation_mapping_node = Node(
        package='elevation_mapping_cupy',
        executable='elevation_mapping_node',
        name='elevation_mapping_node',
        output='screen',
        parameters=[
            ParameterFile(core_param_path, allow_substs=True), 
            sensor_param_path,
            {'use_sim_time': use_sim_time}
        ],
        condition=UnlessCondition(use_python_node)
        # condition=UnlessCondition(PythonExpression(use_python_node))
        )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        use_python_node_arg,
        elevation_mapping_node_py,
        elevation_mapping_node,
        rviz_node
    ])
