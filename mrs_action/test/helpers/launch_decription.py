import launch
import launch_ros
import launch_testing.actions

def generate_test_description(action_parameters: dict[str,str], service_parameters: dict[str,str]):

    ld = launch.LaunchDescription()

    if action_parameters:
        am_node = launch_ros.actions.Node(
                name="action_manager",
                package="mrs_action",
                namespace="test",
                executable="mrs_action_server",
                output="screen",
                parameters=[action_parameters],
            )
        ld.add_action(am_node)

    if service_parameters:
        fake_service_node = launch_ros.actions.Node(
                name="fake_goto",
                package='mrs_action',
                namespace='test',
                executable='fake_goto_node.py',
                output="screen",
                parameters=[service_parameters],
            )
        ld.add_action(fake_service_node)

    ld.add_action(launch.actions.TimerAction(period=1.0, actions=[launch_testing.actions.ReadyToTest()]))

    return ld, {}

