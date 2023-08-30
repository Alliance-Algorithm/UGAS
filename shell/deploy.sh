#!/bin/bash


# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..


# configuration
REMOTE_IP="192.168.234.1"
REMOTE_PORT="22"
REMOTE_USERNAME="root"
REMOTE_PASSWORD="alliance"
REMOTE_DEPLOY_PATH="ugas"


# parse arguments
while [ "$1" != "" ]; do
    case "$1" in
        "--local" )
            REMOTE_IP="127.0.0.1" ;;
        "--build-arg" )
            shift
            break ;;
        * )
            echo "error: $1: invaild option"
            echo "avaliable options:"
            echo "    '--local' will deploy the program to local docker container"
            echo "    '--build-arg' will pass all subsequent arguments to build.sh"
            exit 1 ;;
    esac
    shift
done


# sync file & compile
echo "uploading to remote host..."    
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/rsync -avh -e "/bin/ssh -o StrictHostKeyChecking=no -p ${REMOTE_PORT}" --exclude-from="./.gitignore" . ${REMOTE_USERNAME}@${REMOTE_IP}:${REMOTE_DEPLOY_PATH} --delete-after
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/ssh -o StrictHostKeyChecking=no -p ${REMOTE_PORT} ${REMOTE_USERNAME}@${REMOTE_IP} "${REMOTE_DEPLOY_PATH}/shell/build.sh $* && ~/init.sh --restart"
