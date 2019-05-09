^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package neonavigation_launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2019-05-09)
------------------
* trajectory_tracker: remove unused parameters (`#274 <https://github.com/at-wat/neonavigation/issues/274>`_)
* Contributors: Yuta Koga

0.3.1 (2019-01-10)
------------------
* neonavigation_launch: add dependency to trajectory_tracker_rviz_plugins (`#259 <https://github.com/at-wat/neonavigation/issues/259>`_)
* trajectory_tracker: support PathWithVelocity (`#244 <https://github.com/at-wat/neonavigation/issues/244>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* Contributors: Atsushi Watanabe, So Jomura

0.2.3 (2018-07-19)
------------------
* Install sample files and nodes for demo (`#201 <https://github.com/at-wat/neonavigation/issues/201>`_)
* Contributors: Atsushi Watanabe

0.2.2 (2018-07-17)
------------------

0.2.1 (2018-07-14)
------------------

0.2.0 (2018-07-12)
------------------
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* costmap_cspace: fix layer order handling from the parameter (`#154 <https://github.com/at-wat/neonavigation/issues/154>`_)
* Add integration demo (`#133 <https://github.com/at-wat/neonavigation/issues/133>`_)
* costmap_cspace: add unknown handler layer (`#132 <https://github.com/at-wat/neonavigation/issues/132>`_)
* safety_limiter: add watchdog timer (`#123 <https://github.com/at-wat/neonavigation/issues/123>`_)
* safety_limiter: use timer instead of spinOnce (`#121 <https://github.com/at-wat/neonavigation/issues/121>`_)
* planner_cspace: support parallel aster search (`#118 <https://github.com/at-wat/neonavigation/issues/118>`_)
* costmap_cspace: make static layers configurable (`#108 <https://github.com/at-wat/neonavigation/issues/108>`_)
* costmap_cspace: make costmap layer structure configurable (`#106 <https://github.com/at-wat/neonavigation/issues/106>`_)
* planner_cspace: add simple action client for robot patrol. (`#61 <https://github.com/at-wat/neonavigation/issues/61>`_)
* neonavigation_launch, planner_cspace: add simple simulator. (`#59 <https://github.com/at-wat/neonavigation/issues/59>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* changes planner and costmap package names with a postfix _cspace
* neonavigation_launch: updates trajectory_tracker parameters
* neonavigation_launch: specifies parameter to control yaw at the goal
* neonavigation_launch: changes in-place rotation threshold
* neonavigation_launch: adds arg to specify path looking ahead range
* neonavigation_launch: removes tolerance parameters to use default value
* neonavigation_launch: adds parameter to specify goal topic
* neonavigation_launch: removes unused parameters
* neonavigation_launch: adds args to set planner and costmap parameters
* neonavigation_launch: add args for simulation
* neonavigation_launch: add launch file to use costmap_3d and planner_3d
* Contributors: Atsushi Watanabe
