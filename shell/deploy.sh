#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

REMOTE_IP="192.168.234.1"
REMOTE_PORT="22"
REMOTE_USERNAME="alliance"
REMOTE_PASSWORD="alliance"
REMOTE_PASSWORD_ROOT="alliance"

echo "uploading to remote host..."
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/rsync -av -e "/bin/ssh -p ${REMOTE_PORT}" --exclude-from='./.gitignore' . ${REMOTE_USERNAME}@${REMOTE_IP}:/home/${REMOTE_USERNAME}/ugas
/bin/sshpass -p "${REMOTE_PASSWORD}" /bin/ssh ${REMOTE_USERNAME}@${REMOTE_IP} "/home/${REMOTE_USERNAME}/ugas/shell/build.sh"

if [ "$1" = "--force" ] || [ "$1" = "-f" ]
then
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "/home/${REMOTE_USERNAME}/ugas/shell/install.sh"
else
    if [ "$1" = "" ]
    then
    /bin/sshpass -p "${REMOTE_PASSWORD_ROOT}" /bin/ssh root@${REMOTE_IP} "systemctl restart ugas.service || /home/${REMOTE_USERNAME}/ugas/shell/install.sh"
    else
        echo "warning: $1: invaild option."
        echo "avaliable option: '--force' or '-f' to force installation."
    fi
fi
