^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package track_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.2 (2021-11-08)
-------------------

0.11.1 (2021-10-29)
-------------------

0.11.0 (2021-08-30)
-------------------

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
* Increase initialization timeout in the tests (`#536 <https://github.com/at-wat/neonavigation/issues/536>`_)
* Contributors: Atsushi Watanabe

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
* Fix duplicated tf timestamp (`#494 <https://github.com/at-wat/neonavigation/issues/494>`_)
* track_odometry: increase transform timeout in tests (`#490 <https://github.com/at-wat/neonavigation/issues/490>`_)
* Contributors: Atsushi Watanabe

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
* track_odometry: add option to align all postures to source frame (`#447 <https://github.com/at-wat/neonavigation/issues/447>`_)
* Support Noetic (`#461 <https://github.com/at-wat/neonavigation/issues/461>`_)
* track_odometry: add enable_tcp_no_delay option to reduce latency (`#456 <https://github.com/at-wat/neonavigation/issues/456>`_)
* track_odometry: use double instead of float (`#455 <https://github.com/at-wat/neonavigation/issues/455>`_)
* Contributors: Atsushi Watanabe, Naotaka Hatao, Yuta Koga

0.8.1 (2020-03-12)
------------------
* track_odometry: increase queue sizes of message_filters::Subscriber (`#450 <https://github.com/at-wat/neonavigation/issues/450>`_)
* Contributors: Naotaka Hatao

0.8.0 (2020-03-04)
------------------

0.7.0 (2020-02-04)
------------------

0.6.0 (2020-01-18)
------------------

0.5.1 (2020-01-06)
------------------
* track_odometry: fix test stability (`#412 <https://github.com/at-wat/neonavigation/issues/412>`_)
* Fix header namespaces (`#408 <https://github.com/at-wat/neonavigation/issues/408>`_)
* Migrate from C math functions to C++ (`#407 <https://github.com/at-wat/neonavigation/issues/407>`_)
* Contributors: Atsushi Watanabe

0.5.0 (2019-10-21)
------------------

0.4.3 (2019-09-10)
------------------

0.4.2 (2019-08-19)
------------------

0.4.1 (2019-08-15)
------------------
* track_odometry: synchronize Odometry and IMU (`#363 <https://github.com/at-wat/neonavigation/issues/363>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#310 <https://github.com/at-wat/neonavigation/issues/310>`_)
* Fix include directory priority (`#308 <https://github.com/at-wat/neonavigation/issues/308>`_)
* Contributors: Atsushi Watanabe

0.4.0 (2019-05-09)
------------------
* track_odometry: fix z_filter unit to seconds (`#290 <https://github.com/at-wat/neonavigation/issues/290>`_)
* track_odometry: add project_posture option to tf_projection node (`#286 <https://github.com/at-wat/neonavigation/issues/286>`_)
* track_odometry: refactor tf_projection (`#285 <https://github.com/at-wat/neonavigation/issues/285>`_)
* track_odometry: set missing child_frame_id in tf_projection (`#283 <https://github.com/at-wat/neonavigation/issues/283>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.3.1 (2019-01-10)
------------------
* Fix pointer alignment style (`#233 <https://github.com/at-wat/neonavigation/issues/233>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* track_odometry: add test (`#208 <https://github.com/at-wat/neonavigation/issues/208>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* Contributors: Atsushi Watanabe, So Jomura

0.2.3 (2018-07-19)
------------------
* Fix test names (`#202 <https://github.com/at-wat/neonavigation/issues/202>`_)
* Contributors: Atsushi Watanabe

0.2.2 (2018-07-17)
------------------

0.2.1 (2018-07-14)
------------------

0.2.0 (2018-07-12)
------------------
* Fix build on Indigo (`#180 <https://github.com/at-wat/neonavigation/issues/180>`_)
* Fix namespace migration messages (`#174 <https://github.com/at-wat/neonavigation/issues/174>`_)
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Fix package dependencies (`#167 <https://github.com/at-wat/neonavigation/issues/167>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* track_odometry: use timer instead of spinOnce (`#122 <https://github.com/at-wat/neonavigation/issues/122>`_)
* track_odometry: fix eigen include dir (`#115 <https://github.com/at-wat/neonavigation/issues/115>`_)
* track_odometry: use position diff instead of twist.linear (`#113 <https://github.com/at-wat/neonavigation/issues/113>`_)
* track_odometry: overwrite odometry child_frame_id (`#112 <https://github.com/at-wat/neonavigation/issues/112>`_)
* track_odometry: fix naming style. (`#91 <https://github.com/at-wat/neonavigation/issues/91>`_)
* track_odometry: add publish_tf option. (`#78 <https://github.com/at-wat/neonavigation/issues/78>`_)
* Remove dummy dep to system_lib. (`#51 <https://github.com/at-wat/neonavigation/issues/51>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* Fix coding styles. (`#39 <https://github.com/at-wat/neonavigation/issues/39>`_)
* track_odometry: adds an option to use without odometry input (`#30 <https://github.com/at-wat/neonavigation/issues/30>`_)
* track_odometry: tf_projection: adds parameter to add tf timestamp offset (`#21 <https://github.com/at-wat/neonavigation/issues/21>`_)
* track_odometry: tf_projection: adds option to eliminate roll/pitch (`#20 <https://github.com/at-wat/neonavigation/issues/20>`_)
* track_odometry: refactors tf_projection test code (`#19 <https://github.com/at-wat/neonavigation/issues/19>`_)
* track_odometry: removes projected tf output and add tf_projection node (`#17 <https://github.com/at-wat/neonavigation/issues/17>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* track_odometry: suppresses warnings until receiving first message
* track_odometry: implements kalman filter (`#9 <https://github.com/at-wat/neonavigation/issues/9>`_)
* track_odometry: fixes delta time and buffering
* track_odometry: uses latest transform between imu and base_link
* Subtree-merge 'track_odometry' package
* Contributors: Atsushi Watanabe
