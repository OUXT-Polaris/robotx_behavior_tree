# robotx_behavior_tree

Package including behavior action nodes.

Action nodes for robotx should be inherit [ActionNode](https://github.com/OUXT-Polaris/robotx_behavior_tree/blob/ed98e90355ac5e31fe194e9498cfea4cd08a95c8/robotx_behavior_tree/include/robotx_behavior_tree/action_node.hpp#L29-L41) or [ActionROS2Node](https://github.com/OUXT-Polaris/robotx_behavior_tree/blob/ed98e90355ac5e31fe194e9498cfea4cd08a95c8/robotx_behavior_tree/include/robotx_behavior_tree/action_node.hpp#L43-L97)

In ActionROS2Node, these ports below are always exists and it is unnecessary to write these ports because [robotx_bt_planner node add these ports to xml description automatically.](https://github.com/OUXT-Polaris/robotx_behavior_tree/blob/39b498e1614143fb320be29b27b098b06ed01e35/robotx_bt_planner/src/bt_planner_component.cpp#L223-L242)

| name           | type                                                          | getter                             | description                                                       |
| -------------- | ------------------------------------------------------------- | ---------------------------------- | ----------------------------------------------------------------- |
| task_objects   | robotx_behavior_msgs::msg::TaskObjectsArrayStamped::SharedPtr | ActionROS2Node::getTaskObjects()   | task object data from '/perception/task_objects' topic            |
| planner_status | hermite_path_msgs::msg::PlannerStatus::SharedPtr              | ActionROS2Node::getPlannerStatus() | planner status from '/local_waypoint_server/planner_status' topic |
| current_pose   | geometry_msgs::msg::PoseStamped::SharedPtr                    | ActionROS2Node::getCurrentPose()   | current pose of  the robot.                                       |