^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package planner_cspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.10.11 (2021-06-21)
--------------------
* planner_cspace: reset next_replan_time after waitUntil() (`#602 <https://github.com/at-wat/neonavigation/issues/602>`_)
* Contributors: Kazuki Takahashi

0.10.10 (2021-03-18)
--------------------
* planner_cspace: fix goal unreachable condition (`#595 <https://github.com/at-wat/neonavigation/issues/595>`_)
* Contributors: Atsushi Watanabe

0.10.9 (2021-03-16)
-------------------
* planner_cspace: abort A* search on continuous timeout (`#592 <https://github.com/at-wat/neonavigation/issues/592>`_)
* Contributors: Atsushi Watanabe

0.10.8 (2021-03-10)
-------------------
* planner_cspace: improve performance of costmap reset (`#587 <https://github.com/at-wat/neonavigation/issues/587>`_)
* Contributors: Naotaka Hatao

0.10.7 (2021-03-07)
-------------------
* planner_cspace: improve performance of hysteresis clearing (`#586 <https://github.com/at-wat/neonavigation/issues/586>`_)
* Contributors: Naotaka Hatao

0.10.6 (2021-01-28)
-------------------
* planner_cspace: fix condition of open queue (`#576 <https://github.com/at-wat/neonavigation/issues/576>`_)
* planner_cspace: add debug output about cost_estim_cache update (`#577 <https://github.com/at-wat/neonavigation/issues/577>`_)
* planner_cspace: reuse open/erase queue (`#575 <https://github.com/at-wat/neonavigation/issues/575>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.10.5 (2020-12-24)
-------------------
* planner_cspace: enable tolerance in make_plan (`#570 <https://github.com/at-wat/neonavigation/issues/570>`_)
* Contributors: Naotaka Hatao

0.10.4 (2020-11-12)
-------------------

0.10.3 (2020-10-22)
-------------------

0.10.2 (2020-10-07)
-------------------

0.10.1 (2020-08-26)
-------------------
* planner_cspace: avoid publishing invalid path when escaping (`#546 <https://github.com/at-wat/neonavigation/issues/546>`_)
* Contributors: Naotaka Hatao

0.10.0 (2020-08-06)
-------------------
* planner_cspace: add test to MoveWithToleranceAction (`#528 <https://github.com/at-wat/neonavigation/issues/528>`_)
* planner_cspace: add test to distance map debug output (`#526 <https://github.com/at-wat/neonavigation/issues/526>`_)
* planner_cspace: periodically update local map in the test (`#522 <https://github.com/at-wat/neonavigation/issues/522>`_)
* Merge rostest coverage profiles (`#520 <https://github.com/at-wat/neonavigation/issues/520>`_)
* planner_cspace: fix search range of minimum cost in fast_map_update mode (`#518 <https://github.com/at-wat/neonavigation/issues/518>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.9.1 (2020-07-16)
------------------

0.9.0 (2020-07-02)
------------------

0.8.8 (2020-06-15)
------------------
* planner_cspace: avoid showing too many warning messages (`#501 <https://github.com/at-wat/neonavigation/issues/501>`_)
* Contributors: Naotaka Hatao

0.8.7 (2020-05-22)
------------------

0.8.6 (2020-05-15)
------------------
* Fix duplicated tf timestamp (`#494 <https://github.com/at-wat/neonavigation/issues/494>`_)
* planner_cspace: add wait to navigation tests (`#492 <https://github.com/at-wat/neonavigation/issues/492>`_)
* planner_cspace: simplify path switch detection condition (`#488 <https://github.com/at-wat/neonavigation/issues/488>`_)
* planner_cspace: fix uninitialized variable (`#486 <https://github.com/at-wat/neonavigation/issues/486>`_)
* planner_cspace: enable replan when robot reaches the switchback point (`#449 <https://github.com/at-wat/neonavigation/issues/449>`_)
* planner_cspace: fix test_debug_outputs initial wait (`#485 <https://github.com/at-wat/neonavigation/issues/485>`_)
* Contributors: Atsushi Watanabe, Kazuki Takahashi

0.8.5 (2020-05-04)
------------------

0.8.4 (2020-04-30)
------------------
* Clean unused dependencies (`#472 <https://github.com/at-wat/neonavigation/issues/472>`_)
* Contributors: Atsushi Watanabe

0.8.3 (2020-04-26)
------------------

0.8.2 (2020-04-07)
------------------
* Support Noetic (`#461 <https://github.com/at-wat/neonavigation/issues/461>`_)
* Contributors: Atsushi Watanabe

0.8.1 (2020-03-12)
------------------
* planner_cspace: fix flaky debug_output test (`#452 <https://github.com/at-wat/neonavigation/issues/452>`_)
* planner_cspace: fix condition of planning finish (`#451 <https://github.com/at-wat/neonavigation/issues/451>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.8.0 (2020-03-04)
------------------
* planner_cspace: replan immediately when path is blocked by new obstacles (`#446 <https://github.com/at-wat/neonavigation/issues/446>`_)
* Add message package version constraints (`#443 <https://github.com/at-wat/neonavigation/issues/443>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.7.0 (2020-02-04)
------------------
* planner_cspace: add MoveWithToleranceAction server (`#433 <https://github.com/at-wat/neonavigation/issues/433>`_)
* planner_cspace: fix typo (`#436 <https://github.com/at-wat/neonavigation/issues/436>`_)
* planner_cspace: implement motion primitive algorithm for speed-up (`#431 <https://github.com/at-wat/neonavigation/issues/431>`_)
* Contributors: Daiki Maekawa, Naotaka Hatao

0.6.0 (2020-01-18)
------------------

0.5.1 (2020-01-06)
------------------
* planner_cspace: disable blockmem_gridmap_performance test (`#413 <https://github.com/at-wat/neonavigation/issues/413>`_)
* Fix header namespaces (`#408 <https://github.com/at-wat/neonavigation/issues/408>`_)
* planner_cspace: fix installing planner_2dof_serial_joints node (`#409 <https://github.com/at-wat/neonavigation/issues/409>`_)
* Migrate from C math functions to C++ (`#407 <https://github.com/at-wat/neonavigation/issues/407>`_)
* planner_cspace: split search model definition (`#323 <https://github.com/at-wat/neonavigation/issues/323>`_)
* planner_cspace: fix debug output test (`#404 <https://github.com/at-wat/neonavigation/issues/404>`_)
* planner_cspace: fix navigation test stability (`#403 <https://github.com/at-wat/neonavigation/issues/403>`_)
* planner_cspace: add planner_2dof_serial_joints node test (`#402 <https://github.com/at-wat/neonavigation/issues/402>`_)
* Contributors: Atsushi Watanabe

0.5.0 (2019-10-21)
------------------
* planner_cspace: fix debug output test stability (`#399 <https://github.com/at-wat/neonavigation/issues/399>`_)
* planner_cspace: publish internally used maps as OccupancyGrid (`#396 <https://github.com/at-wat/neonavigation/issues/396>`_)
* planner_cspace: clear hysteresis if new obstacle is on the previous path (`#393 <https://github.com/at-wat/neonavigation/issues/393>`_)
* planner_cspace: fix remember_updates feature (`#391 <https://github.com/at-wat/neonavigation/issues/391>`_)
* Contributors: Atsushi Watanabe

0.4.3 (2019-09-10)
------------------
* planner_cspace: make sure that planner error will be cleared if the goal is aborted (`#372 <https://github.com/at-wat/neonavigation/issues/372>`_)
* Contributors: Daiki Maekawa

0.4.2 (2019-08-19)
------------------
* planner_cspace: fix planner performance (`#369 <https://github.com/at-wat/neonavigation/issues/369>`_)
* Contributors: Atsushi Watanabe

0.4.1 (2019-08-15)
------------------
* planner_cspace: fix debug build compatibility (`#368 <https://github.com/at-wat/neonavigation/issues/368>`_)
* planner_cspace: fix out-of-boundary validation (`#362 <https://github.com/at-wat/neonavigation/issues/362>`_)
* planner_cspace: fix incomplete output path after search timeout (`#357 <https://github.com/at-wat/neonavigation/issues/357>`_)
* planner_cspace: reduce position quantization error on planning (`#351 <https://github.com/at-wat/neonavigation/issues/351>`_)
* planner_cspace: latch publish data in navigation test (`#353 <https://github.com/at-wat/neonavigation/issues/353>`_)
* planner_cspace: improve grid search performance (`#342 <https://github.com/at-wat/neonavigation/issues/342>`_)
* planner_cspace: optimize BlockmemGridmap (`#315 <https://github.com/at-wat/neonavigation/issues/315>`_)
* planner_cspace: add a launch for planner performance evaluation (`#343 <https://github.com/at-wat/neonavigation/issues/343>`_)
* planner_cspace: fix parallel memory access (`#306 <https://github.com/at-wat/neonavigation/issues/306>`_)
* planner_cspace: remove hist mode of debug output (`#336 <https://github.com/at-wat/neonavigation/issues/336>`_)
* planner_cspace: fix navigation test setup (`#335 <https://github.com/at-wat/neonavigation/issues/335>`_)
* planner_cspace: add a navigation test case with map update (`#334 <https://github.com/at-wat/neonavigation/issues/334>`_)
* planner_cspace: add const to the end pos (`#332 <https://github.com/at-wat/neonavigation/issues/332>`_)
* planner_cspace: reject request if input frame are located at diffrent frame to the map (`#327 <https://github.com/at-wat/neonavigation/issues/327>`_)
* planner_cspace: publish empty path immediately after planning aborted (`#326 <https://github.com/at-wat/neonavigation/issues/326>`_)
* planner_cspace: revert default sw_wait parameter (`#313 <https://github.com/at-wat/neonavigation/issues/313>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#310 <https://github.com/at-wat/neonavigation/issues/310>`_)
* planner_cspace: calculate path hysteresis in 3-DOF space (`#304 <https://github.com/at-wat/neonavigation/issues/304>`_)
* Fix include directory priority (`#308 <https://github.com/at-wat/neonavigation/issues/308>`_)
* planner_cspace: fix CyclicVector dimension of planner_2dof_serial_joints (`#307 <https://github.com/at-wat/neonavigation/issues/307>`_)
* planner_cspace, costmap_cspace: minor refactoring (`#305 <https://github.com/at-wat/neonavigation/issues/305>`_)
* Fix empty path publish (`#301 <https://github.com/at-wat/neonavigation/issues/301>`_)
* planner_cspace: refactor CyclicVec (`#300 <https://github.com/at-wat/neonavigation/issues/300>`_)
* planner_cspace: refactor rotation cache (`#299 <https://github.com/at-wat/neonavigation/issues/299>`_)
* planner_cspace: fix path cost calculation and interpolation (`#298 <https://github.com/at-wat/neonavigation/issues/298>`_)
* Contributors: Atsushi Watanabe, Daiki Maekawa, Yuta Koga

0.4.0 (2019-05-09)
------------------
* planner_cspace: limit negative cost to avoid infinite search loop (`#288 <https://github.com/at-wat/neonavigation/issues/288>`_)
* trajectory_tracker: remove unused parameters (`#274 <https://github.com/at-wat/neonavigation/issues/274>`_)
* Support melodic (`#266 <https://github.com/at-wat/neonavigation/issues/266>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.3.1 (2019-01-10)
------------------
* trajectory_tracker: support PathWithVelocity (`#244 <https://github.com/at-wat/neonavigation/issues/244>`_)
* planner_cspace: fix stability of test_costmap_watchdog (`#242 <https://github.com/at-wat/neonavigation/issues/242>`_)
* planner_cspace: add watchdog to costmap update (`#235 <https://github.com/at-wat/neonavigation/issues/235>`_)
* planner_cspace: add missing test dependencies (`#234 <https://github.com/at-wat/neonavigation/issues/234>`_)
* Fix pointer alignment style (`#233 <https://github.com/at-wat/neonavigation/issues/233>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* planner_cspace: add diagnostics to planner node (`#226 <https://github.com/at-wat/neonavigation/issues/226>`_)
* planner_cspace: stop robot motion if new map received (`#218 <https://github.com/at-wat/neonavigation/issues/218>`_)
* planner_cspace: split grid-metric converter functions (`#213 <https://github.com/at-wat/neonavigation/issues/213>`_)
* planner_cspace: split motion cache class (`#212 <https://github.com/at-wat/neonavigation/issues/212>`_)
* planner_cspace: fix goal and start tolerance parameter (`#211 <https://github.com/at-wat/neonavigation/issues/211>`_)
* planner_cspace: add cost for turning near obstacles (`#210 <https://github.com/at-wat/neonavigation/issues/210>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* planner_cspace: use odometry position difference in jump detection (`#205 <https://github.com/at-wat/neonavigation/issues/205>`_)
* planner_cspace: refactoring (`#204 <https://github.com/at-wat/neonavigation/issues/204>`_)
* Contributors: Atsushi Watanabe, So Jomura, Yuta Koga

0.2.3 (2018-07-19)
------------------
* Fix test names (`#202 <https://github.com/at-wat/neonavigation/issues/202>`_)
* Contributors: Atsushi Watanabe

0.2.2 (2018-07-17)
------------------

0.2.1 (2018-07-14)
------------------
* Fix missing package dependencies (`#194 <https://github.com/at-wat/neonavigation/issues/194>`_)
* Contributors: Atsushi Watanabe

0.2.0 (2018-07-12)
------------------
* planner_cspace: fix restriction of path segment connection (`#191 <https://github.com/at-wat/neonavigation/issues/191>`_)
* planner_cspace: fix boundary check (`#190 <https://github.com/at-wat/neonavigation/issues/190>`_)
* planner_cspace: fix unconverged switching back vibration (`#183 <https://github.com/at-wat/neonavigation/issues/183>`_)
* Reduce random test failure (`#181 <https://github.com/at-wat/neonavigation/issues/181>`_)
* Update CI (`#179 <https://github.com/at-wat/neonavigation/issues/179>`_)
* Fix cost in heuristic function for make_plan service (`#178 <https://github.com/at-wat/neonavigation/issues/178>`_)
* Fix namespace migration messages (`#174 <https://github.com/at-wat/neonavigation/issues/174>`_)
* planner_cspace: add make plan service (`#169 <https://github.com/at-wat/neonavigation/issues/169>`_)
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Fix package dependencies (`#167 <https://github.com/at-wat/neonavigation/issues/167>`_)
* Fix naming styles (`#166 <https://github.com/at-wat/neonavigation/issues/166>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* planner_cspace: fix clearing remembered costmap (`#158 <https://github.com/at-wat/neonavigation/issues/158>`_)
* planner_cspace: fix partial costmap update with unknown cells (`#156 <https://github.com/at-wat/neonavigation/issues/156>`_)
* planner_cspace: remember costmap using binary bayes filter (`#149 <https://github.com/at-wat/neonavigation/issues/149>`_)
* planner_cspace: fix position jump detection (`#150 <https://github.com/at-wat/neonavigation/issues/150>`_)
* planner_cspace: fix remembering costmap (`#147 <https://github.com/at-wat/neonavigation/issues/147>`_)
* planner_cspace: use frame_id of incoming message to set dummy robot pose (`#145 <https://github.com/at-wat/neonavigation/issues/145>`_)
* planner_cspace: add odom publisher to dummy robot (`#143 <https://github.com/at-wat/neonavigation/issues/143>`_)
* planner_cspace: add preempt (`#137 <https://github.com/at-wat/neonavigation/issues/137>`_)
* planner_cspace: minor optimizations (`#129 <https://github.com/at-wat/neonavigation/issues/129>`_)
* planner_cspace: disable performance test by default (`#127 <https://github.com/at-wat/neonavigation/issues/127>`_)
* planner_cspace: support parallel distance map search (`#125 <https://github.com/at-wat/neonavigation/issues/125>`_)
* planner_cspace: support parallel aster search (`#118 <https://github.com/at-wat/neonavigation/issues/118>`_)
* Add abort (`#116 <https://github.com/at-wat/neonavigation/issues/116>`_)
* planner_cspace: increase navigation test time limit (`#98 <https://github.com/at-wat/neonavigation/issues/98>`_)
* planner_cspace: validate goal position. (`#90 <https://github.com/at-wat/neonavigation/issues/90>`_)
* Suppress compile warnings and test with -Werror. (`#82 <https://github.com/at-wat/neonavigation/issues/82>`_)
* Fix header of empty path. (`#79 <https://github.com/at-wat/neonavigation/issues/79>`_)
* planner_cspace: cache motion interpolation. (`#75 <https://github.com/at-wat/neonavigation/issues/75>`_)
* planner_cspace: add planning performance test. (`#74 <https://github.com/at-wat/neonavigation/issues/74>`_)
* planner_cspace: add navigation integration test. (`#73 <https://github.com/at-wat/neonavigation/issues/73>`_)
* planner_cspace: add test for cyclic_vec. (`#72 <https://github.com/at-wat/neonavigation/issues/72>`_)
* planner_cspace: fix naming styles in blockmem_gridmap. (`#69 <https://github.com/at-wat/neonavigation/issues/69>`_)
* planner_cspace: add test for blockmem_gridmap. (`#70 <https://github.com/at-wat/neonavigation/issues/70>`_)
* planner_cspace: install patrol actionlib client. (`#64 <https://github.com/at-wat/neonavigation/issues/64>`_)
* planner_cspace: initialize dummy robot status. (`#62 <https://github.com/at-wat/neonavigation/issues/62>`_)
* planner_cspace: add simple action client for robot patrol. (`#61 <https://github.com/at-wat/neonavigation/issues/61>`_)
* planner_cspace: add missing dependency to boost::chrono. (`#60 <https://github.com/at-wat/neonavigation/issues/60>`_)
* planner_cspace: add actionlib support. (`#58 <https://github.com/at-wat/neonavigation/issues/58>`_)
* neonavigation_launch, planner_cspace: add simple simulator. (`#59 <https://github.com/at-wat/neonavigation/issues/59>`_)
* planner_space: fix naming styles. (`#57 <https://github.com/at-wat/neonavigation/issues/57>`_)
* planner_cspace: refactor separating classes. (`#55 <https://github.com/at-wat/neonavigation/issues/55>`_)
* planner_cspace: fix distance map init timing. (`#53 <https://github.com/at-wat/neonavigation/issues/53>`_)
* Remove dummy dep to system_lib. (`#51 <https://github.com/at-wat/neonavigation/issues/51>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* Fix coding styles. (`#39 <https://github.com/at-wat/neonavigation/issues/39>`_)
* planner_cspace: fixes ignore range handling (`#28 <https://github.com/at-wat/neonavigation/issues/28>`_)
* planner_cspace: fixes memory leak on remembered costmap (`#27 <https://github.com/at-wat/neonavigation/issues/27>`_)
* planner_cspace: adds service to forget remembered costmap (`#26 <https://github.com/at-wat/neonavigation/issues/26>`_)
* planner_cspace: fixes logic of remember_update parameter (`#25 <https://github.com/at-wat/neonavigation/issues/25>`_)
* planner_cspace: fixes wrong direction of path end (`#24 <https://github.com/at-wat/neonavigation/issues/24>`_)
* planner_cspace: fixes straight motion discriminant (`#23 <https://github.com/at-wat/neonavigation/issues/23>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* costmap_cspace, planner_cspace: fixes pkg dependencies
* planner_cspace: adds planner for 2dof serial joints (`#6 <https://github.com/at-wat/neonavigation/issues/6>`_)
* planner_cspace: uses template to specify dimension
* changes planner and costmap package names with a postfix _cspace
* Contributors: Atsushi Watanabe, Yuta Koga, Yutaka Takaoka
