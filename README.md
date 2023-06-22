
# UGAS
Universal Gimbal Aiming System

## Build UGAS
```shell
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -DENABLE_DEBUG_CANVAS=OFF -DENABLE_OPENVINO=OFF -DENABLE_RECORDING=OFF -DENABLE_ROS=OFF ..
```

## Build UGAS with ROS2
```shell
mkdir -p ~/ros2_ws/src & cd ~/ros2_ws
cp -r ~/Desktop/UGAS ./src/ugas
colcon build --packages-select ugas --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DENABLE_ROS=ON -GNinja
source ./install/local_setup.bash
ros2 run ugas main
```