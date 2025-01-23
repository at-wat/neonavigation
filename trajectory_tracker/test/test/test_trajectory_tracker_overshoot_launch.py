import unittest
from typing import Tuple

import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ActiveProcInfoHandler


@pytest.mark.launch_test
@launch_testing.markers.keep_alive  # type: ignore[misc]
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    node_log_level = [
        TextSubstitution(text="trajectory_tracker:="),
        LaunchConfiguration("node_log_level", default="info"),
    ]
    params_file = PathJoinSubstitution(
        [
            get_package_share_directory("trajectory_tracker"),  # type: ignore[no-untyped-call]
            "test",
            "configs",
            LaunchConfiguration("param_file", default="test_overshoot_params.yaml"),
        ]
    )
    test_executable = LaunchConfiguration(
        "test_executable", default="test_trajectory_tracker_overshoot"
    )
    trajectory_tracker_cmd = Node(
        package="trajectory_tracker",
        executable="trajectory_tracker",
        name="trajectory_tracker",
        output="screen",
        arguments=["--ros-args", "--log-level", node_log_level],
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    test_cmd = Node(
        package="trajectory_tracker",
        executable=test_executable,
        name=test_executable,
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[params_file, {"use_sim_time": use_sim_time}],
    )
    time_soure_cmd = Node(
        package="trajectory_tracker",
        executable="time_source",
        name="time_source",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[params_file],
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
            process=test_cmd, timeout=60.0
        )
        self.assertEqual(proc_info[test_cmd].returncode, 0)
