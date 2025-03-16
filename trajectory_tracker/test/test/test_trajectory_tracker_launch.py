import os
import unittest
from typing import Tuple

import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ActiveProcInfoHandler
from nav2_common.launch import RewrittenYaml  # type: ignore[attr-defined]


@pytest.mark.launch_test
@launch_testing.markers.keep_alive  # type: ignore[misc]
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    node_log_level = [
        TextSubstitution(text="trajectory_tracker:="),
        LaunchConfiguration("node_log_level", default="info"),
    ]
    params_file = os.path.join(
        get_package_share_directory("trajectory_tracker"),  # type: ignore[no-untyped-call]
        "test",
        "configs",
        "test_params.yaml",
    )
    odom_delay = LaunchConfiguration("odom_delay", default="0.0")
    use_odom = LaunchConfiguration("use_odom", default="false")
    use_time_optimal_control = LaunchConfiguration("use_time_optimal_control", default="true")
    param_substitutions = {
        "odom_delay": odom_delay,
        "use_odom": use_odom,
        "use_time_optimal_control": use_time_optimal_control,
    }
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )
    trajectory_tracker_cmd = Node(
        package="trajectory_tracker",
        executable="trajectory_tracker",
        name="trajectory_tracker",
        output="screen",
        arguments=["--ros-args", "--log-level", node_log_level],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )
    test_cmd = Node(
        package="trajectory_tracker",
        executable="test_trajectory_tracker",
        name="test_trajectory_tracker",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[configured_params, {"use_sim_time": use_sim_time}],
    )
    time_soure_cmd = Node(
        package="trajectory_tracker",
        executable="time_source",
        name="time_source",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[configured_params],
        condition=IfCondition(use_sim_time),
    )

    return LaunchDescription(
        [
            trajectory_tracker_cmd,
            test_cmd,
            time_soure_cmd,
            ReadyToTest(),  # type: ignore[no-untyped-call]
        ]
    ), {"test_cmd": test_cmd}


class TestTerminatingProcessStops(unittest.TestCase):

    def test_proc_terminates(self, proc_info: ActiveProcInfoHandler, test_cmd: Node) -> None:
        proc_info.assertWaitForShutdown(  # type: ignore[no-untyped-call]
            process=test_cmd, timeout=120.0
        )
        self.assertEqual(proc_info[test_cmd].returncode, 0)
