SRC_DIR=$(pwd)
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SRC_DIR}/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:${SRC_DIR}/build/devel/lib:/opt/ros/noetic/lib

