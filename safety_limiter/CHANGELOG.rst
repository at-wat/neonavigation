^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package safety_limiter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

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
