import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))

import launch
import launch.actions
import launch.events

import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

def main(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = launch.LaunchDescription()

    # Prepare the map server node.
    map_server = launch_ros.actions.LifecycleNode(
        name='map_server',
        package='nav2_map_server', executable='map_server', output='screen')

    # When the map server reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_map_server_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=map_server, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'map server' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(map_server),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # Make the map server node take the 'configure' transition.
    emit_event_to_request_that_map_server_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(map_server),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_map_server_reaches_inactive_state)
    ld.add_action(map_server)
    ld.add_action(emit_event_to_request_that_map_server_does_configure_transition)

    # ls = launch.LaunchService(argv=argv, debug=True)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main()