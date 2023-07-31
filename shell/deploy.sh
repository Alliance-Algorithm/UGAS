#!/bin/bash


# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..


DEBUG="OFF"
FORCE_INSTALLATION="OFF"
while [ "$1" != "" ]; do
    case "$1" in
        "--debug" )
            DEBUG="ON" ;;
        "-d" )
            DEBUG="ON" ;;
        "--force" )
            FORCE_INSTALLATION="ON" ;;
        "-f" )
            FORCE_INSTALLATION="ON" ;;
        "--build-arg" )
            shift
            break ;;
        * )
            echo "error: $1: invaild option"
            echo "avaliable options:"
            echo "    '--force' or '-f' to force installation."
            echo "    '--build-arg' will pass all subsequent arguments to build.sh"
            exit 1 ;;
    esac
    shift
done

if [ "${DEBUG}" = "ON" ] && [ "${FORCE_INSTALLATION}" = "ON" ]
then
    echo "error: the option debug and force cannot be enabled at the same time."
    echo "--debug forbids installation and --force forced installation, which is contradictory."
fi


REMOTE_IP="192.168.234.1"
REMOTE_PORT="22"
REMOTE_USERNAME="alliance"
REMOTE_PASSWORD="alliance"
REMOTE_PASSWORD_ROOT="alliance"

if [ "${DEBUG}" = "OFF" ]
then
    REMOTE_DEPLOY_PATH="/home/${REMOTE_USERNAME}/ugas"
else
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "killall ugas || true"
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "systemctl stop ugas.service || true"
    REMOTE_DEPLOY_PATH="/home/${REMOTE_USERNAME}/ugas_debug"
fi


echo "uploading to remote host..."    
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/rsync -av -e "/bin/ssh -p ${REMOTE_PORT}" --exclude-from="./.gitignore" . ${REMOTE_USERNAME}@${REMOTE_IP}:${REMOTE_DEPLOY_PATH}
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/ssh ${REMOTE_USERNAME}@${REMOTE_IP} "${REMOTE_DEPLOY_PATH}/shell/build.sh $*"

if [ "${DEBUG}" = "OFF" ]
then
    if [ "${FORCE_INSTALLATION}" = "ON" ]
    then
        /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "${REMOTE_DEPLOY_PATH}/shell/install.sh"
    else
        /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "systemctl restart ugas.service || ${REMOTE_DEPLOY_PATH}/shell/install.sh"
    fi
fi














    
