# Post-release test script for neonavigation related packages

This script tests shadow-fixed binary release.
planner_cspace, costmap_cspace and trajectory_tracker are tested by the current script.

NVIDIA graphic board is not supported in the script at now.
Use Intel or AMD graphic board.

```shell
$ xhost +
$ ./post-release-test.sh kinetic
$ ./post-release-test.sh melodic
```
