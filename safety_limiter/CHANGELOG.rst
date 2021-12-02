^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package safety_limiter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.11.3 (2021-12-02)
-------------------
* Fix flaky tests and add debug outputs (`#628 <https://github.com/at-wat/neonavigation/issues/628>`_)
* Contributors: Atsushi Watanabe

0.11.2 (2021-11-08)
-------------------

0.11.1 (2021-10-29)
-------------------

0.11.0 (2021-08-30)
-------------------
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
* safety_limiter: add a max_linear_vel and max_angular_vel (`#581 <https://github.com/at-wat/neonavigation/issues/581>`_)
* Contributors: Teo Cardoso

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
* Merge rostest coverage profiles (`#520 <https://github.com/at-wat/neonavigation/issues/520>`_)
* Contributors: Atsushi Watanabe

0.9.1 (2020-07-16)
------------------

0.9.0 (2020-07-02)
------------------
* safety_limiter: make safety_limiter dynamic-reconfigurable (`#509 <https://github.com/at-wat/neonavigation/issues/509>`_)
* Contributors: Naotaka Hatao

0.8.8 (2020-06-15)
------------------

0.8.7 (2020-05-22)
------------------
* [Noetic] Add missing includes and libraries (`#497 <https://github.com/at-wat/neonavigation/issues/497>`_)
* Contributors: Shane Loretz

0.8.6 (2020-05-15)
------------------

0.8.5 (2020-05-04)
------------------
* Revert "Remove old workaround for debian stretch build (`#473 <https://github.com/at-wat/neonavigation/issues/473>`_)" (`#478 <https://github.com/at-wat/neonavigation/issues/478>`_)
* Contributors: Atsushi Watanabe

0.8.4 (2020-04-30)
------------------
* Remove old workaround for debian stretch build (`#473 <https://github.com/at-wat/neonavigation/issues/473>`_)
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
* safety_limiter: use fixed frame for accumulating input cloud (`#421 <https://github.com/at-wat/neonavigation/issues/421>`_)
* Contributors: Yuta Koga

0.5.1 (2020-01-06)
------------------
* safety_limiter: fix test stability (`#411 <https://github.com/at-wat/neonavigation/issues/411>`_)
* Migrate from C math functions to C++ (`#407 <https://github.com/at-wat/neonavigation/issues/407>`_)
* safety_limiter: fix test stability (`#406 <https://github.com/at-wat/neonavigation/issues/406>`_)
* Contributors: Atsushi Watanabe

0.5.0 (2019-10-21)
------------------
* safety_limiter: remove debug output (`#385 <https://github.com/at-wat/neonavigation/issues/385>`_)
* safety_limiter: status broadcasting from safety_limiter node (`#383 <https://github.com/at-wat/neonavigation/issues/383>`_)
* Contributors: Atsushi Watanabe, Daiki Maekawa

0.4.3 (2019-09-10)
------------------
* safety_limiter: fix diagnostics warning condition (`#374 <https://github.com/at-wat/neonavigation/issues/374>`_)
* Contributors: Atsushi Watanabe

0.4.2 (2019-08-19)
------------------

0.4.1 (2019-08-15)
------------------
* safety_limiter: increase simulation test publish rate (`#320 <https://github.com/at-wat/neonavigation/issues/320>`_)
* safety_limiter: add simulation test conditions for backward motion (`#319 <https://github.com/at-wat/neonavigation/issues/319>`_)
* safety_limiter: add delay compensation (`#316 <https://github.com/at-wat/neonavigation/issues/316>`_)
* safety_limiter: fix footprint radius calculation (`#317 <https://github.com/at-wat/neonavigation/issues/317>`_)
* Drop ROS Indigo and Ubuntu Trusty support (`#310 <https://github.com/at-wat/neonavigation/issues/310>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.4.0 (2019-05-09)
------------------
* safety_limiter: fix backward motion limit (`#292 <https://github.com/at-wat/neonavigation/issues/292>`_)
* safety_limiter: fix CloudBuffering test start timing (`#279 <https://github.com/at-wat/neonavigation/issues/279>`_)
* Fix package dependencies (`#268 <https://github.com/at-wat/neonavigation/issues/268>`_)
* Contributors: Atsushi Watanabe, Yuta Koga

0.3.1 (2019-01-10)
------------------
* safety_limiter: increase tolerance of the test (`#243 <https://github.com/at-wat/neonavigation/issues/243>`_)
* Fix pointer alignment style (`#233 <https://github.com/at-wat/neonavigation/issues/233>`_)
* Migrate tf to tf2 (`#230 <https://github.com/at-wat/neonavigation/issues/230>`_)
* safety_limiter: add diagnostics to safety_limiter node  (`#227 <https://github.com/at-wat/neonavigation/issues/227>`_)
* safety_limiter: allow escape motion from collision (`#221 <https://github.com/at-wat/neonavigation/issues/221>`_)
* safety_limiter: fix first time step of collision prediction (`#222 <https://github.com/at-wat/neonavigation/issues/222>`_)
* Fix catkin package definitions (`#206 <https://github.com/at-wat/neonavigation/issues/206>`_)
* Contributors: Atsushi Watanabe, So Jomura, Yuta Koga

0.2.3 (2018-07-19)
------------------

0.2.2 (2018-07-17)
------------------
* Workaround for debian stretch build problem (`#199 <https://github.com/at-wat/neonavigation/issues/199>`_)
* Contributors: Atsushi Watanabe

0.2.1 (2018-07-14)
------------------
* Compile with PCL_NO_PRECOMPILE (`#195 <https://github.com/at-wat/neonavigation/issues/195>`_)
* Contributors: Atsushi Watanabe

0.2.0 (2018-07-12)
------------------
* safety_limiter: update document (`#192 <https://github.com/at-wat/neonavigation/issues/192>`_)
* safety_limiter: fix safety limit logic (`#187 <https://github.com/at-wat/neonavigation/issues/187>`_)
* safety_limiter: fix input cloud buffering (`#184 <https://github.com/at-wat/neonavigation/issues/184>`_)
* Fix namespace migration messages (`#174 <https://github.com/at-wat/neonavigation/issues/174>`_)
* Fix topic/service namespace model (`#168 <https://github.com/at-wat/neonavigation/issues/168>`_)
* Fix package dependencies (`#167 <https://github.com/at-wat/neonavigation/issues/167>`_)
* Fix naming styles (`#166 <https://github.com/at-wat/neonavigation/issues/166>`_)
* Update package descriptions and unify license and version (`#165 <https://github.com/at-wat/neonavigation/issues/165>`_)
* Use neonavigation_msgs package (`#164 <https://github.com/at-wat/neonavigation/issues/164>`_)
* safety_limiter: add watchdog timer (`#123 <https://github.com/at-wat/neonavigation/issues/123>`_)
* safety_limiter: use timer instead of spinOnce (`#121 <https://github.com/at-wat/neonavigation/issues/121>`_)
* safety_limiter: fix naming style. (`#86 <https://github.com/at-wat/neonavigation/issues/86>`_)
* Suppress compile warnings and test with -Werror. (`#82 <https://github.com/at-wat/neonavigation/issues/82>`_)
* safety_limiter: avoid kdtree build for empty cloud. (`#67 <https://github.com/at-wat/neonavigation/issues/67>`_)
* Add missing dep to xmlrpcpp. (`#52 <https://github.com/at-wat/neonavigation/issues/52>`_)
* safety_limiter: support fragmented pointcloud input. (`#43 <https://github.com/at-wat/neonavigation/issues/43>`_)
* Support package install. (`#45 <https://github.com/at-wat/neonavigation/issues/45>`_)
* Fix coding styles. (`#39 <https://github.com/at-wat/neonavigation/issues/39>`_)
* adds READMEs (`#11 <https://github.com/at-wat/neonavigation/issues/11>`_)
* safety_limiter: increases subscribe buffer length for safety disable input
* safety_limiter: adds time margin in collision test
* safety_limiter: uses pcl's erase-remove_if
* safety_limiter: fixes safety disable mode to ignore cloud timeout
* safety_limiter: adds safety disable input
* safety_limiter: fixes pointcloud height handling
* safety_limiter: reduces pointcloud timeout warning
* safety_limiter: publishes stop command if no pointcloud received
* safety_limiter: Motion limiter for collision prevention
* Contributors: Atsushi Watanabe
