^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package costmap_cspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.4 (2022-04-13)
-------------------
* Divide output layer class into specific classes (`#633 <https://github.com/at-wat/neonavigation/issues/633>`_)
* Contributors: Naotaka Hatao

0.11.3 (2021-12-02)
-------------------

0.11.2 (2021-11-08)
-------------------

0.11.1 (2021-10-29)
-------------------

0.11.0 (2021-08-30)
-------------------
* Rename PointcloudAccumurator to PointcloudAccumulator (`#608 <https://github.com/at-wat/neonavigation/issues/608>`_)
* Apply clang-format-11 with new setting (`#605 <https://github.com/at-wat/neonavigation/issues/605>`_)
* Contributors: Naotaka Hatao, f-fl0

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
* costmap_cspace: make Costmap3dLayerPlain and Costmap3dLayerOutput faster (`#562 <https://github.com/at-wat/neonavigation/issues/562>`_)
* Contributors: Naotaka Hatao

0.10.3 (2020-10-22)
-------------------
* costmap_cspace: make Costmap3dLayerFootprint::generateCSpace faster (`#554 <https://github.com/at-wat/neonavigation/issues/554>`_)
* costmap_cspace: add test for keeping unknown mode (`#555 <https://github.com/at-wat/neonavigation/issues/555>`_)
* Contributors: Naotaka Hatao

0.10.2 (2020-10-07)
-------------------
* costmap_cspace: enable func to remove objects out of z range (`#551 <https://github.com/at-wat/neonavigation/issues/551>`_)
* Contributors: Tatsuroh Sakaguchi

0.10.1 (2020-08-26)
-------------------

0.10.0 (2020-08-06)
-------------------

0.9.1 (2020-07-16)
------------------

0.9.0 (2020-07-02)
------------------

0.8.8 (2020-06-15)
------------------

0.8.7 (2020-05-22)
------------------

0.8.6 (2020-05-15)
------------------

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

0.8.0 (2020-03-04)
------------------

0.7.0 (2020-02-04)
------------------

0.6.0 (2020-01-18)
------------------

0.5.1 (2020-01-06)
------------------
* Fix header namespaces (`#408 <https://github.com/at-wat/neonavigation/issues/408>`_)
* Migrate from C math functions to C++ (`#407 <https://github.com/at-wat/neonavigation/issues/407>`_)
* Contributors: Atsushi Watanabe

0.5.0 (2019-10-21)
------------------
* costmap_cspace: fix unknown handling (`#392 <https://github.com/at-wat/neonavigation/issues/392>`_)
* Contributors: Atsushi Watanabe

0.4.3 (2019-09-10)
------------------
* costmap_cspace: fix memory corruption (`#381 <https://github.com/at-wat/neonavigation/issues/381>`_)
* Contributors: Atsushi Watanabe

0.4.2 (2019-08-19)
------------------

0.4.1 (2019-08-15)
------------------
* costmap_cspace: fix costmap output for out-of-boundary overlay map (`#361 <https://github.com/at-wat/neonavigation/issues/361>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#310 <https://github.com/at-wat/neonavigation/issues/310>`_)
* Fix include directory priority (`#308 <https://github.com/at-wat/neonavigation/issues/308>`_)
* planner_cspace, costmap_cspace: minor refactoring (`#305 <https://github.com/at-wat/neonavigation/issues/305>`_)
* Contributors: Atsushi Watanabe

0.4.0 (2019-05-09)
------------------
* costmap_cspace: add const begin/end to PointcloudAccumurator (`#294 <https://github.com/at-wat/neonavigation/issues/294>`_)
* Contributors: Naotaka Hatao

0.3.1 (2019-01-10)
------------------
* Fix pointer alignment style (`#233 <https://github.com/at-wat/neonavigation/issues/233>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* Contributors: Atsushi Watanabe, So Jomura

0.2.3 (2018-07-19)
------------------
* Fix test names (`#202 <https://github.com/at-wat/neonavigation/issues/202>`_)
* Install sample files and nodes for demo (`#201 <https://github.com/at-wat/neonavigation/issues/201>`_)
* Contributors: Atsushi Watanabe

0.2.2 (2018-07-17)
------------------

0.2.1 (2018-07-14)
------------------
* Fix missing package dependencies (`#194 <https://github.com/at-wat/neonavigation/issues/194>`_)
* Contributors: Atsushi Watanabe

0.2.0 (2018-07-12)
------------------
* Fix build on Indigo (`#180 <https://github.com/at-wat/neonavigation/issues/180>`_)
* Fix namespace migration messages (`#174 <https://github.com/at-wat/neonavigation/issues/174>`_)
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Fix package dependencies (`#167 <https://github.com/at-wat/neonavigation/issues/167>`_)
* Fix naming styles (`#166 <https://github.com/at-wat/neonavigation/issues/166>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* costmap_cspace: add StopPropagation layer (`#153 <https://github.com/at-wat/neonavigation/issues/153>`_)
* costmap_cspace: install header files (`#155 <https://github.com/at-wat/neonavigation/issues/155>`_)
* costmap_cspace: fix layer order handling from the parameter (`#154 <https://github.com/at-wat/neonavigation/issues/154>`_)
* costmap_cspace: clear update region on output layer (`#139 <https://github.com/at-wat/neonavigation/issues/139>`_)
* costmap_cspace: clear previous position on overlay map (`#135 <https://github.com/at-wat/neonavigation/issues/135>`_)
* Add integration demo (`#133 <https://github.com/at-wat/neonavigation/issues/133>`_)
* costmap_cspace: add unknown handler layer (`#132 <https://github.com/at-wat/neonavigation/issues/132>`_)
* costmap_cspace: memory access optimizations (`#131 <https://github.com/at-wat/neonavigation/issues/131>`_)
* costmap_cspace: keep overlay maps which arrived before base map (`#124 <https://github.com/at-wat/neonavigation/issues/124>`_)
* costmap_cspace: always store received overlay map (`#109 <https://github.com/at-wat/neonavigation/issues/109>`_)
* costmap_cspace: make static layers configurable (`#108 <https://github.com/at-wat/neonavigation/issues/108>`_)
* costmap_cspace: make costmap layer structure configurable (`#106 <https://github.com/at-wat/neonavigation/issues/106>`_)
* costmap_cspace: refactor CSpace3Cache class (`#105 <https://github.com/at-wat/neonavigation/issues/105>`_)
* costmap_cspace: add stacked costmap class (`#104 <https://github.com/at-wat/neonavigation/issues/104>`_)
* costmap_cspace: remove retval of setFootprint (`#103 <https://github.com/at-wat/neonavigation/issues/103>`_)
* costmap_cspace: move XML-polygon conversion into Polygon class (`#102 <https://github.com/at-wat/neonavigation/issues/102>`_)
* costmap cspace: add layer type check (`#101 <https://github.com/at-wat/neonavigation/issues/101>`_)
* costmap_cspace: refactor costmap layer classes (`#100 <https://github.com/at-wat/neonavigation/issues/100>`_)
* costmap_cspace: make costmap layers stackable (`#99 <https://github.com/at-wat/neonavigation/issues/99>`_)
* costmap_cspace: make cspace template customizable (`#96 <https://github.com/at-wat/neonavigation/issues/96>`_)
* costmap_cspace: hold CSpace3D object as shared_ptr (`#95 <https://github.com/at-wat/neonavigation/issues/95>`_)
* Suppress compile warnings and test with -Werror. (`#82 <https://github.com/at-wat/neonavigation/issues/82>`_)
* costmap_cspace: fix frame of z-filter in pointcloud2_to_map. (`#80 <https://github.com/at-wat/neonavigation/issues/80>`_)
* costmap_cspace: fix angular grid accessor before receiving the first map.
* Add missing dep to xmlrpcpp. (`#52 <https://github.com/at-wat/neonavigation/issues/52>`_)
* Remove dummy dep to system_lib. (`#51 <https://github.com/at-wat/neonavigation/issues/51>`_)
* costmap_cspace: adds unit tests. (`#48 <https://github.com/at-wat/neonavigation/issues/48>`_)
* costmap_cspace: fixes memory access error on global map boundary. (`#49 <https://github.com/at-wat/neonavigation/issues/49>`_)
* costmap_cspace: refactors costmap_cspace package. (`#47 <https://github.com/at-wat/neonavigation/issues/47>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* costmap_cspace: pointcloud2_to_map: adds singleshot data input (`#41 <https://github.com/at-wat/neonavigation/issues/41>`_)
* Fix coding styles. (`#39 <https://github.com/at-wat/neonavigation/issues/39>`_)
* costmap_cspace: add pointcloud2_to_map node. (`#35 <https://github.com/at-wat/neonavigation/issues/35>`_)
* costmap_cspace: laserscan_to_map: accumerate scans. (`#34 <https://github.com/at-wat/neonavigation/issues/34>`_)
* costmap_cspace: adds laserscan_to_map node. (`#33 <https://github.com/at-wat/neonavigation/issues/33>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* costmap_cspace, planner_cspace: fixes pkg dependencies
* changes planner and costmap package names with a postfix _cspace
* Contributors: Atsushi Watanabe
