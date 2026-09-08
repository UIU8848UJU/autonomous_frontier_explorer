0.0.5 (2026-09-08)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Make the package buildable with standard CMake and no ROS 2 or Nav2 dependency.
* Move ROS messages, logging, time, costmap, footprint, reachability and provider
  facades into the separate ``frontier_explorer_ros`` adapter package.
* Add a permanent package-boundary test that rejects ROS/Nav2 dependencies.

0.0.4 (2026-09-07)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add the ROS-free ``frontier_explorer_selection_core`` target.
* Move frontier pruning, retry/blacklist state and candidate-pool selection
  behind pure C++ ``FrontierPruner`` and ``FrontierSelectionPolicy`` APIs.
* Keep Nav2 costmap and footprint checks in the ROS-facing selector facade.
* Preserve planner reachability diagnostic reasons in scored candidates.
* Replace the old ROS-bound ``FrontierPruner`` constructor and call contract;
  this pre-1.0 internal API change requires downstream sources to migrate and rebuild.

0.0.3 (2026-09-06)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

* Add the ROS-free ``GridMap`` and ``frontier_explorer_detection_core`` target.
* Add the ROS-free ``frontier_explorer_scoring_core`` target.
* Replace the old ``FrontierDetector`` ROS/OccupancyGrid overloads with the
  explicit ``CostmapAdapter::gridMap()`` seam.
* Remove the ``rclcpp::Logger`` constructor argument from ``FrontierScorer``.
  Downstream callers must migrate to the pure C++ constructors.
