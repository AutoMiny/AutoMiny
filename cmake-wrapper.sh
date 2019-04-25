#!/bin/bash
CATKIN_PARAMS=""
# Check if CMAKE_BUILD_TYPE is present (only used when loading the project)
if [[ $1 == *"CMAKE_BUILD_TYPE"* ]]; then
	CATKIN_PARAMS="-DCMAKE_INSTALL_PREFIX=../install -DCATKIN_DEVEL_PREFIX=../devel"
fi

cmake $CATKIN_PARAMS "$@"