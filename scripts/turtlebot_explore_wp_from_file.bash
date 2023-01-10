#!/bin/bash

# load waypoints into parameter server
echo loading waypoints
rosparam load `rospack find inspection`/config/waypoints.yaml;

# let RoadmapServer know that wps are available in param server
rosservice call /rosplan_roadmap_server/load_waypoints;

# add robot turtlebot instance
echo "Adding initial state and goals to knowledge base.";
param_type="update_type:
- 0";
param="knowledge:
- knowledge_type: 0
  instance_type: 'robot'
  instance_name: 'turtlebot'
  attribute_name: ''
  function_value: 0.0";
param_type="$param_type
- 0";
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'robot_at'
  values:
  - {key: 'v', value: 'turtlebot'}
  - {key: 'wp', value: 'wp0'}
  function_value: 0.0";
param_type="$param_type
- 0";
param="$param
- knowledge_type: 1
  instance_type: ''
  instance_name: ''
  attribute_name: 'charge_at'
  values:
  - {key: 'wp', value: 'wp2'}
  function_value: 0.0";

rosservice call /rosplan_knowledge_base/update_array "
$param_type
$param"

# NOTE: robot_at(turtlebot wp0) gets added by the mapping interface 


# call turtlebot_explore_common.bash script
bash `rospack find inspection`/scripts/turtlebot_explore_common.bash
