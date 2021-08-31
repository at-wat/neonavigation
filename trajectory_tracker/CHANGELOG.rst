^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajectory_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.0 (2021-08-30)
-------------------
* trajectory_tracker: add velocity tolerance parameters (`#607 <https://github.com/at-wat/neonavigation/issues/607>`_)
* Apply clang-format-11 with new setting (`#605 <https://github.com/at-wat/neonavigation/issues/605>`_)
* Contributors: Naotaka Hatao

0.10.11 (2021-06-21)
--------------------

0.10.10 (2021-03-18)
--------------------

0.10.9 (2021-03-16)
-------------------

0.10.8 (2021-03-10)
-------------------

0.10.7 (2021-03-07)
-------------------

0.10.6 (2021-01-28)
-------------------

0.10.5 (2020-12-24)
-------------------

0.10.4 (2020-11-12)
-------------------

0.10.3 (2020-10-22)
-------------------

0.10.2 (2020-10-07)
-------------------

0.10.1 (2020-08-26)
-------------------

0.10.0 (2020-08-06)
-------------------
* trajectory_tracker: add a mode to apply the same control method during turning in place (`#513 <https://github.com/at-wat/neonavigation/issues/513>`_)
* trajectory_tracker: relax test tolerance on tf mode (`#545 <https://github.com/at-wat/neonavigation/issues/545>`_)
* trajectory_tracker: goal if both raw and predicted pose is in tolerance (`#540 <https://github.com/at-wat/neonavigation/issues/540>`_)
* trajectory_tracker: fix wrong tracking target just after new path is received (`#537 <https://github.com/at-wat/neonavigation/issues/537>`_)
* Increase initialization timeout in the tests (`#536 <https://github.com/at-wat/neonavigation/issues/536>`_)
* trajectory_tracker: add odometry timeout checking (`#534 <https://github.com/at-wat/neonavigation/issues/534>`_)
* trajectory_tracker: predict odometry by extrapolation (`#529 <https://github.com/at-wat/neonavigation/issues/529>`_)
* trajectory_tracker: add use_odom option (`#523 <https://github.com/at-wat/neonavigation/issues/523>`_)
* trajectory_tracker: make trajectory_tracker dynamic-reconfigurable (`#521 <https://github.com/at-wat/neonavigation/issues/521>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.9.1 (2020-07-16)
------------------
* trajectory_tracker: fix remained distance calculation on overshoot (`#514 <https://github.com/at-wat/neonavigation/issues/514>`_)
* Contributors: Atsushi Watanabe

0.9.0 (2020-07-02)
------------------

0.8.8 (2020-06-15)
------------------

0.8.7 (2020-05-22)
------------------

0.8.6 (2020-05-15)
------------------
* Fix duplicated tf timestamp (`#494 <https://github.com/at-wat/neonavigation/issues/494>`_)
* Contributors: Atsushi Watanabe

0.8.5 (2020-05-04)
------------------

0.8.4 (2020-04-30)
------------------
* Clean unused dependencies (`#472 <https://github.com/at-wat/neonavigation/issues/472>`_)
* trajectory_tracker: add missing dep to std_srvs (`#470 <https://github.com/at-wat/neonavigation/issues/470>`_)
* Contributors: Atsushi Watanabe

0.8.3 (2020-04-26)
------------------

0.8.2 (2020-04-07)
------------------
* Support Noetic (`#461 <https://github.com/at-wat/neonavigation/issues/461>`_)
* Contributors: Atsushi Watanabe

0.8.1 (2020-03-12)
------------------

0.8.0 (2020-03-04)
------------------
* Add message package version constraints (`#443 <https://github.com/at-wat/neonavigation/issues/443>`_)
* trajectory_tracker: check path timestamps in tests (`#441 <https://github.com/at-wat/neonavigation/issues/441>`_)
* trajectory_tracker: add path header to TrajectoryTrackerStatus (`#439 <https://github.com/at-wat/neonavigation/issues/439>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.7.0 (2020-02-04)
------------------
* trajectory_tracker: calculate correct curvature at the end of path (`#435 <https://github.com/at-wat/neonavigation/issues/435>`_)
* trajectory_tracker: fix test initialization timeout (`#432 <https://github.com/at-wat/neonavigation/issues/432>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao

0.6.0 (2020-01-18)
------------------
* trajectory_tracker: add a service to clear recorded path (`#422 <https://github.com/at-wat/neonavigation/issues/422>`_)
* Contributors: Naotaka Hatao

0.5.1 (2020-01-06)
------------------
* Migrate from C math functions to C++ (`#407 <https://github.com/at-wat/neonavigation/issues/407>`_)
* trajectory_tracker: fix test stability (`#405 <https://github.com/at-wat/neonavigation/issues/405>`_)
* Contributors: Atsushi Watanabe

0.5.0 (2019-10-21)
------------------
* trajectory_tracker: track interpolated rotation (`#394 <https://github.com/at-wat/neonavigation/issues/394>`_)
* Contributors: Atsushi Watanabe

0.4.3 (2019-09-10)
------------------

0.4.2 (2019-08-19)
------------------

0.4.1 (2019-08-15)
------------------
* trajectory_tracker: update demo params (`#352 <https://github.com/at-wat/neonavigation/issues/352>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#310 <https://github.com/at-wat/neonavigation/issues/310>`_)
* planner_cspace: calculate path hysteresis in 3-DOF space (`#304 <https://github.com/at-wat/neonavigation/issues/304>`_)
* Fix include directory priority (`#308 <https://github.com/at-wat/neonavigation/issues/308>`_)
* Contributors: Atsushi Watanabe

0.4.0 (2019-05-09)
------------------
* trajectory_tracker: speed up simulation on rostest (`#280 <https://github.com/at-wat/neonavigation/issues/280>`_)
* trajectory_tracker: linear velocity adaptive gain control (`#276 <https://github.com/at-wat/neonavigation/issues/276>`_)
* trajectory_tracker: remove unused parameters (`#274 <https://github.com/at-wat/neonavigation/issues/274>`_)
* trajectory_tracker: fix remained distance for path with two poses (`#272 <https://github.com/at-wat/neonavigation/issues/272>`_)
* Add LICENSE file (`#270 <https://github.com/at-wat/neonavigation/issues/270>`_)
* Support melodic (`#266 <https://github.com/at-wat/neonavigation/issues/266>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.3.1 (2019-01-10)
------------------
* trajectory_tracker: fix test timeout for path with velocity (`#263 <https://github.com/at-wat/neonavigation/issues/263>`_)
* trajectory_tracker: add timeout to the test (`#261 <https://github.com/at-wat/neonavigation/issues/261>`_)
* trajectory_tracker: support PathWithVelocity (`#244 <https://github.com/at-wat/neonavigation/issues/244>`_)
* trajectory_tracker: fix robot pose prediction (`#250 <https://github.com/at-wat/neonavigation/issues/250>`_)
* trajectory_tracker: fix angular velocity limit (`#252 <https://github.com/at-wat/neonavigation/issues/252>`_)
* trajectory_tracker: add acceleration factor parameter of time optimal control (`#249 <https://github.com/at-wat/neonavigation/issues/249>`_)
* trajectory_tracker: fix local goal handling (`#251 <https://github.com/at-wat/neonavigation/issues/251>`_)
* trajectory_tracker: add tolerance to InPlaceTurn (`#248 <https://github.com/at-wat/neonavigation/issues/248>`_)
* trajectory_tracker: fix angle normalization in in-place turn mode (`#247 <https://github.com/at-wat/neonavigation/issues/247>`_)
* trajectory_tracker: refactoring (`#239 <https://github.com/at-wat/neonavigation/issues/239>`_)
* Fix pointer alignment style (`#233 <https://github.com/at-wat/neonavigation/issues/233>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* trajectory_tracker: fix status output topic path (`#225 <https://github.com/at-wat/neonavigation/issues/225>`_)
* trajectory_tracker: add tests (`#207 <https://github.com/at-wat/neonavigation/issues/207>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* Contributors: Atsushi Watanabe, So Jomura

0.2.3 (2018-07-19)
------------------

0.2.2 (2018-07-17)
------------------

0.2.1 (2018-07-14)
------------------

0.2.0 (2018-07-12)
------------------
* Fix namespace migration messages (`#174 <https://github.com/at-wat/neonavigation/issues/174>`_)
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Fix package dependencies (`#167 <https://github.com/at-wat/neonavigation/issues/167>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* trajectory_tracker: reduce angular oscillation (`#120 <https://github.com/at-wat/neonavigation/issues/120>`_)
* trajectory_tracker: use timer instead of spinOnce polling (`#119 <https://github.com/at-wat/neonavigation/issues/119>`_)
* trajectory_tracker: fix naming style. (`#92 <https://github.com/at-wat/neonavigation/issues/92>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* Fix coding styles. (`#39 <https://github.com/at-wat/neonavigation/issues/39>`_)
* trajectory_tracker: removes unnecessary launch files (`#18 <https://github.com/at-wat/neonavigation/issues/18>`_)
* trajectory_tracker: adds option to store timestamp in recorded path (`#13 <https://github.com/at-wat/neonavigation/issues/13>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* trajectory_tracker: subtree merge changes on trajectory_tracker repository
* Subtree-merge 'trajectory_tracker' package
* Contributors: Atsushi Watanabe
