^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package planner_cspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
