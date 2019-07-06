#!/bin/sh

exec rosservice call --wait /planner_3d/set_logger_level "logger: 'ros.planner_cspace'
level: '${1:-info}'"
