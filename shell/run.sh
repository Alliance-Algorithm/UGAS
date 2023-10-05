#!/bin/bash

# exit when any error occurs
set -e

# cd current directionary to ugas root
cd "${0%/*}"
cd ..

source shell/env_init.sh

echo "executing ugas..."
build/ugas
