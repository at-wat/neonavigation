^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package track_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
