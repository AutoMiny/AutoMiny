import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro

def generate_launch_description():
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzserver.launch.py']),
        launch_arguments={
            'world': os.path.join(get_package_share_directory('autominy_sim'), 'worlds/empty.world')
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gzclient.launch.py']),
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

    spawn_world = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/world/robot_description',
                                   '-entity', 'lab_model',
                                   '-x', '2.890656',
                                   '-y', '1.951848',
                                   '-z', '0.0001',
                                   '-Y', '3.14159'],
                        output='screen')

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', '/robot_description',
                                   '-entity', 'model_car',
                                   '-x', '0.189021',
                                   '-y', '4.092613',
                                   '-z', '0.05',
                                   '-Y', '-1.5708'],
                        output='screen')

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
        gzserver,
        gzclient,
        spawn_world,
        spawn_entity,
    ])