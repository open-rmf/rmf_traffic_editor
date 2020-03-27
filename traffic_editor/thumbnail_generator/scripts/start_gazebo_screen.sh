#!/bin/bash

export SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

if [ "$#" -ne 1 ]; then
  echo "usage: ./run_gazebo.sh COLOR"
  echo "  where COLOR is either 'green' or 'white'"
  echo "  (both sets of thumbnails need to be generated)"
  exit
fi

COLOR=$1
echo "generating images with background color: [$COLOR]"

# todo: be smarter about calculating paths. use rospack and stuff.
mkdir -p $SCRIPT_DIR/../images/$COLOR

. /opt/ros/melodic/setup.bash
. /usr/share/gazebo/setup.sh

# append the paths that are required
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH

export WORLD_PATH=$SCRIPT_DIR/../worlds/top_view_$COLOR.world

roslaunch gazebo_ros empty_world.launch verbose:=true world_name:=$WORLD_PATH
