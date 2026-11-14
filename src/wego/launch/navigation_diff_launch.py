import os
import re
import tempfile
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, EnvironmentVariable
from launch_ros.substitutions import FindPackageShare

from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


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


def launch_setup(context):
    wego_share_dir = get_package_share_directory('wego')
    wego_nav_share_dir = get_package_share_directory('wego_2d_nav')

    # robot namespace (e.g. limo_a) -> nav2 runs under /limo_a (/limo_a/navigate_to_pose ...)
    namespace = LaunchConfiguration('namespace').perform(context).strip('/')
    prefix = '/' + namespace if namespace else ''

    # setting for rviz configuration path
    rviz_file_name = 'navigation.rviz'
    rviz_config_path = os.path.join(wego_share_dir, 'rviz', rviz_file_name)

    # set the parameters
    parameter_file_name = 'diff_navigation_params.yaml'
    parameter_file_path = os.path.join(wego_nav_share_dir, 'params', parameter_file_name)

    # put the parameters under the namespace and point the costmaps at /<namespace>/scan
    # (humble costmap resolves relative observation topics inside the costmap node namespace)
    namespaced_params = RewrittenYaml(
        source_file=parameter_file_path,
        root_key=namespace,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.obstacle_layer.scan.topic': prefix + '/scan',
            'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': prefix + '/scan',
        },
        convert_types=True,
    )

    # set the map yaml file
    map_file_name = 'map.yaml'
    map_file_path = os.path.join(wego_nav_share_dir, 'maps', map_file_name)

    # remapping tf topic
    remappings = [('/tf', prefix + '/tf'), ('/tf_static', prefix + '/tf_static')]

    # set container for composable node
    nav2_container = Node(
        name='nav2_container',
        package='rclcpp_components',
        executable='component_container_isolated',
        parameters=[ParameterFile(namespaced_params), {'autostart': True}],
        arguments=['--ros-args', '--log-level', 'info'],
        remappings=remappings,
        output='screen',
    )

    # For localization
    localization_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('wego_2d_nav'),
            'launch',
            'localization_launch.py',
        ]),
        launch_arguments={
            'map' : map_file_path,
            'params_file': namespaced_params,
            'container_name': prefix + '/nav2_container',
        }.items()
    )

    # For navigation
    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('wego_2d_nav'),
            'launch',
            'navigation_only_launch.py',
        ]),
        launch_arguments={
            'params_file': namespaced_params,
            'container_name': prefix + '/nav2_container',
        }.items()
    )

    # setting for rviz
    rviz_config_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', namespaced_rviz_config(rviz_config_path, prefix)],
            remappings=remappings,
        )

    actions = [PushRosNamespace(namespace)] if namespace else []
    actions += [
        SetRemap(src='/tf', dst=prefix + '/tf'),
        SetRemap(src='/tf_static', dst=prefix + '/tf_static'),
        nav2_container,
        localization_launch,
        navigation_launch,
        rviz_config_node,
    ]

    return [GroupAction(actions)]


def generate_launch_description():
    namespace_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value=EnvironmentVariable('LIMO_NAMESPACE', default_value=''),
        description='Robot namespace (e.g. limo_a). Defaults to $LIMO_NAMESPACE'
    )

    return LaunchDescription([
        namespace_launch_arg,
        OpaqueFunction(function=launch_setup),
    ])
