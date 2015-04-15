#!/bin/bash -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${WORKSPACE_DIR}"

echo "VM will be shut down."
read -p "Press CTRL-C to abort, Enter to continue: "

vagrant halt
VBoxManage export qtcopter_dev -o qtcopter_dev.ova
