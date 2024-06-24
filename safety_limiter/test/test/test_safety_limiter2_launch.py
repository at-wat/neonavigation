import os
import unittest
from typing import Tuple

import launch_testing.markers
import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_testing.actions import ReadyToTest
from launch_testing.proc_info_handler import ActiveProcInfoHandler


@pytest.mark.launch_test
@launch_testing.markers.keep_alive  # type: ignore[misc]
def generate_test_description() -> Tuple[LaunchDescription, dict[str, LaunchDescriptionEntity]]:
    node_log_level = [
        TextSubstitution(text="safety_limiter:="),
        LaunchConfiguration("node_log_level", default="info"),
    ]
    params_file = os.path.join(
        get_package_share_directory("safety_limiter"),  # type: ignore[no-untyped-call]
        "test",
        "configs",
        "test_params2.yaml",
    )
    safety_limiter_cmd = Node(
        package="safety_limiter",
        executable="safety_limiter",
        name="safety_limiter",
        output="screen",
        arguments=["--ros-args", "--log-level", node_log_level],
        parameters=[params_file],
    )
    test_cmd = Node(
        package="safety_limiter",
        executable="test_safety_limiter2",
        name="test_safety_limiter2",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[params_file],
    )

    return LaunchDescription(
        [
            safety_limiter_cmd,
            test_cmd,
            ReadyToTest(),  # type: ignore[no-untyped-call]
        ]
    ), {"test_cmd": test_cmd}


class TestTerminatingProcessStops(unittest.TestCase):

    def test_proc_terminates(self, proc_info: ActiveProcInfoHandler, test_cmd: Node) -> None:
        proc_info.assertWaitForShutdown(  # type: ignore[no-untyped-call]
            process=test_cmd, timeout=120.0
        )
        self.assertEqual(proc_info[test_cmd].returncode, 0)
