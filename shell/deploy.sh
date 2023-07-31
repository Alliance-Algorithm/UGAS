#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

FORCE_INSTALLATION="OFF"
while [ "$1" != "" ]; do
    case "$1" in
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

REMOTE_IP="192.168.234.1"
REMOTE_PORT="22"
REMOTE_USERNAME="alliance"
REMOTE_PASSWORD="alliance"
REMOTE_PASSWORD_ROOT="alliance"

echo "uploading to remote host..."
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/rsync -av -e "/bin/ssh -p ${REMOTE_PORT}" --exclude-from='./.gitignore' . ${REMOTE_USERNAME}@${REMOTE_IP}:/home/${REMOTE_USERNAME}/ugas
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/ssh ${REMOTE_USERNAME}@${REMOTE_IP} "/home/${REMOTE_USERNAME}/ugas/shell/build.sh $@"

if [ "${FORCE_INSTALLATION}" == "ON" ]
then
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "/home/${REMOTE_USERNAME}/ugas/shell/install.sh"
else
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "systemctl restart ugas.service || /home/${REMOTE_USERNAME}/ugas/shell/install.sh"
fi

