^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package trajectory_tracker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
