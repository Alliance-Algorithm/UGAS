#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

source shell/env_init.sh

echo "building ugas..."
mkdir -p build && cd build
cmake -GNinja -DCMAKE_BUILD_TYPE=Release -DENABLE_DEBUG_CANVAS=OFF -DENABLE_OPENVINO=OFF -DENABLE_RECORDING=OFF -DENABLE_ROS=OFF ..
ninja

cd ..
