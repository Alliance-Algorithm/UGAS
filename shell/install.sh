#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

echo "installing ugas..."

tee /lib/systemd/system/ugas.service <<_EOF_
[Unit]
Description=Universal Gimbal Aiming System

[Service]
ExecStart=${PWD}/shell/run.sh
Type=simple
KillMode=control-group
Restart=on-failure
RestartSec=30s

[Install]
WantedBy=multi-user.target

_EOF_

systemctl daemon-reload
systemctl enable ugas.service
systemctl restart ugas.service
systemctl status ugas.service

