import os
import re
import tempfile

import yaml

import launch
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable

from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def namespaced_rviz_config(rviz_config_path, prefix):
    # prefix every absolute topic in the rviz config with the robot namespace
    if not prefix:
        return rviz_config_path
    with open(rviz_config_path, 'r') as f:
        text = f.read()
    text = re.sub(r'^(\s*(?:Value|Topic): )/(?=\w)', r'\1' + prefix + '/', text, flags=re.M)
    out = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    out.write(text)
    out.close()
    return out.name


def namespace_free_params(yaml_path, node_name):
    # re-key the node parameters with '/**' so they apply regardless of namespace
    with open(yaml_path, 'r') as f:
        params = yaml.safe_load(f)[node_name]['ros__parameters']
    out = tempfile.NamedTemporaryFile(mode='w', suffix='.yaml', delete=False)
    yaml.dump({'/**': {'ros__parameters': params}}, out)
    out.close()
    return out.name


def launch_setup(context):
    wego_share_dir = get_package_share_directory('wego')
    rviz_config_path = os.path.join(wego_share_dir, 'rviz', 'display.rviz')

    # robot namespace (e.g. limo_a) -> all topics / tf are placed under /limo_a
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = '/' + namespace if namespace else ''

    ydlidar_params = namespace_free_params(
        os.path.join(get_package_share_directory('ydlidar_ros2_driver'), 'params', 'ydlidar.yaml'),
        'ydlidar_ros2_driver_node')
    ekf_params = namespace_free_params(
        os.path.join(get_package_share_directory('robot_localization'), 'params', 'limo_ekf.yaml'),
        'ekf_filter_node_odom')

    actions = []
    if namespace:
        actions.append(PushRosNamespace(namespace))

    actions += [
        # every node publishes / listens tf on /<namespace>/tf
        SetRemap(src='/tf', dst=prefix + '/tf'),
        SetRemap(src='/tf_static', dst=prefix + '/tf_static'),

        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('limo_description'), 'launch', 'load_urdf.launch.py'])
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('wego'), 'launch', 'camera_tilt_launch.py']),
            launch_arguments={
                'degree': LaunchConfiguration('degree')
            }.items()
        ),

        # same as limo_base/launch/limo_base.launch.py, but limo_base uses absolute topic
        # names (/odom, /cmd_vel ...), so they are remapped into the namespace here
        Node(
            package='limo_base',
            executable='limo_base',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'port_name': 'ttylimo',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom',
                'pub_odom_tf': False,
                'control_rate': 50,
            }],
            remappings=[
                ('/odom', prefix + '/odom'),
                ('/imu', prefix + '/imu'),
                ('/cmd_vel', prefix + '/cmd_vel'),
                ('/limo_status', prefix + '/limo_status'),
            ],
            namespace='limo'
        ),

        # orbbec driver publishes extrinsics on absolute topics
        SetRemap(src='/camera/depth_to_color', dst=prefix + '/camera/depth_to_color'),
        SetRemap(src='/camera/depth_to_ir', dst=prefix + '/camera/depth_to_ir'),
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('orbbec_camera'), 'launch', 'astra_stereo_u3.launch.py'])
        ),

        # ydlidar.launch.py forces namespace '/', so the node is declared here instead
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[ydlidar_params],
        ),

        # same as robot_localization/launch/limo_ekf_launch.py, but namespace aware
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[ekf_params],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(LaunchConfiguration('viz')),
            arguments=['-d', namespaced_rviz_config(rviz_config_path, prefix)]
        ),
    ]

    return [GroupAction(actions)]


def generate_launch_description():
    degree_launch_arg = DeclareLaunchArgument(
        'degree',
        default_value='0.0'
    )

    viz_launch_arg = DeclareLaunchArgument(
        'viz',
        default_value='false'
    )

    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('LIMO_NAMESPACE', default_value=''),
        description='Robot namespace (e.g. limo_a). Defaults to $LIMO_NAMESPACE'
    )

    return launch.LaunchDescription([
        degree_launch_arg,
        viz_launch_arg,
        namespace_launch_arg,
        OpaqueFunction(function=launch_setup),
    ])
