#!/bin/bash

# NOTE: this file should only be used in docker images.

SCREEN_SOCKNAME="ugas"
EXECUTABLE_PATH="${HOME}/ugas/build/ugas.sh"

if [[ "$1" == "--entry" ]]; then
    # act as entry point of docker, should be executed by docker.
    set -e
    service ssh restart
    if [ -f "${EXECUTABLE_PATH}" ]; then
        screen -dmS "${SCREEN_SOCKNAME}" "$0" "--daemon"
        echo "Program launched successfully. Type 'screen -r ${SCREEN_SOCKNAME}' to monitor output."
    else
        echo "Program to launch is not found."
    fi
    # sleep infinity to avoid container be terminated.
    sleep infinity
elif [[ "$1" == "--daemon" ]]; then
    # act as daemon, should be executed by the script itself.
    while true; do
        echo "Launching program..."
        "${EXECUTABLE_PATH}"
        echo "Process crashed with return code $?! Will try to restart after 5 seconds."
        sleep 5
    done
elif [[ "$1" == "--restart" ]]; then
    # restart the program, should be executed by ssh when hot-reloading.
    set -e
    if screen -X -S "${SCREEN_SOCKNAME}" quit; then
        echo "Waiting for program stop..."
        while screen -list "${SCREEN_SOCKNAME}" > /dev/null; do
            screen -wipe > /dev/null || true
            sleep 1
        done
    fi
    screen -dmS "${SCREEN_SOCKNAME}" "$0" "--daemon"
    echo "Program restarted successfully."
else
    echo "Error: $1: Invaild option."
fi
