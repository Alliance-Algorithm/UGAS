#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

# parse arguments
CLEAR_BEFORE_BUILD="OFF"
RUN_AFTER_BUILD="OFF"
ENABLE_DEBUG_CANVAS="OFF"
ENABLE_OPENVINO="OFF"
ENABLE_RECORDING="OFF"
ENABLE_ROS="OFF"
while [ "$1" != "" ]; do
    case "$1" in
        "--clear" )
            CLEAR_BEFORE_BUILD="ON" ;;
        "-c" )
            CLEAR_BEFORE_BUILD="ON" ;;
        "--run" )
            RUN_AFTER_BUILD="ON" ;;
        "-r" )
            RUN_AFTER_BUILD="ON" ;;
        "--debug-canvas" )
            ENABLE_DEBUG_CANVAS="ON" ;;
        "--openvino" )
            ENABLE_OPENVINO="ON" ;;
        "--record" )
            ENABLE_RECORDING="ON" ;;
        "--ros" )
            ENABLE_ROS="ON" ;;
        * )
            echo "error: $1: invaild option"
            echo "avaliable options:"
            echo "    '--clear' or '-c' clear cmake cache before compliation."
            echo "    '--run' or '-r' to run ugas after compliation."
            echo "    '--debug-canvas' to enable display of debug images."
            echo "    '--openvino' to enable openvino detection."
            echo "    '--record' to enable runtime image recording."
            echo "    '--ros' to enable ros2 compilation."
            exit 1 ;;
    esac
    shift
done

# source rc file (optional)
LOCAL_INIT_FILE="${HOME}/.ugasrc"
if [ -f "${LOCAL_INIT_FILE}" ]; then source "${LOCAL_INIT_FILE}"; fi

# clear compile output before build (optional)
if [ "${CLEAR_BEFORE_BUILD}" = "ON" ]
then
    echo "clearing cache..."
    rm -rf ./build
    rm -rf ./install
    rm -rf ./log
fi

echo "building ugas..."

# compile
if [ "${ENABLE_ROS}" = "OFF" ]
then
    mkdir -p ./build && cd ./build
    cmake -GNinja -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_DEBUG_CANVAS=${ENABLE_DEBUG_CANVAS} -DENABLE_OPENVINO=${ENABLE_OPENVINO} -DENABLE_RECORDING=${ENABLE_RECORDING} -DENABLE_ROS=OFF ..
    ninja
    cd ..
    echo -e "\x23\x21/bin/bash\ncd \"\${0%/*}/..\"\nif [ -f \"${LOCAL_INIT_FILE}\" ]; then source ${LOCAL_INIT_FILE}; fi\n./build/ugas" > ./build/ugas.sh
else
    colcon build --packages-select ugas --event-handlers console_direct+ --cmake-args -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release \
        -DENABLE_DEBUG_CANVAS=${ENABLE_DEBUG_CANVAS} -DENABLE_OPENVINO=${ENABLE_OPENVINO} -DENABLE_RECORDING=${ENABLE_RECORDING} -DENABLE_ROS=ON
    echo -e "\x23\x21/bin/bash\ncd \"\${0%/*}/..\"\nif [ -f \"${LOCAL_INIT_FILE}\" ]; then source ${LOCAL_INIT_FILE}; fi\nsource ./install/local_setup.bash\nros2 run ugas main" > ./build/ugas.sh
fi

# execute (optional)
chmod u+x ./build/ugas.sh
if [ "${RUN_AFTER_BUILD}" = "ON" ]
then
    echo "executing ugas..."
    ./build/ugas.sh
fi
