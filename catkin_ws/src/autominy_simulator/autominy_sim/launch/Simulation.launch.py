import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

def generate_launch_description():
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world = os.path.join(get_package_share_directory('autominy_sim'), 'worlds/empty.world')
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        #launch_arguments={
        #    'world': os.path.join(get_package_share_directory('autominy_sim'), 'worlds/empty.world'),
        #    'params_file': os.path.join(get_package_share_directory('autominy'), 'params/gazebo.yaml')
        #}.items(),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )


    xacro_file = os.path.join(get_package_share_directory('world_description'),
                              'urdf/circuits/lab.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        name='world_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        remappings=[('robot_description', '/world/robot_description'),
                    ('joint_states', '/world/joint_states')]
    )

    spawn_world = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/world/robot_description',
                                   '-name', 'lab_model',
                                   '-x', '2.890656',
                                   '-y', '1.951848',
                                   '-z', '0.0001',
                                   '-Y', '3.14159'],
                        output='screen')

    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-name', 'model_car',
                                   '-x', '0.189021',
                                   '-y', '4.092613',
                                   '-z', '0.05',
                                   '-Y', '-1.5708'],
                        output='screen')

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("autominy_sim"),
            "config",
            "control.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        namespace="rrbot",
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_car_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'sim_car_controller'],
        output='screen'
    )

    bridge_params = os.path.join(
        get_package_share_directory('sim_car_controller'),
        'config',
        'gz_bridge.yaml'
    )

    gazebo_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )


    tf2_world_lab = Node(name= "tf2_world_lab",
                         package = "tf2_ros",
                         executable = "static_transform_publisher",
                         arguments = ["2.35", "2.891", "0.0001", "-1.5708", "0", "0", "world", "lab_base_link"])

    tf2_world_map = Node(name= "tf2_world_map",
                         package = "tf2_ros",
                         executable = "static_transform_publisher",
                         arguments = ["0", "0", "0", "0", "0", "0", "world", "map"])

    tf2_map_odom = Node(name= "tf2_map_odom",
                         package = "tf2_ros",
                         executable = "static_transform_publisher",
                         arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"])



    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_car_controller],
            )
        ),
        node_robot_state_publisher,
        tf2_world_lab,
        tf2_world_map,
        tf2_map_odom,
        gazebo_ros_bridge,
        gzserver,
        gzclient,
        spawn_world,
        spawn_entity,
    ])