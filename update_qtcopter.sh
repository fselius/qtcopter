#!/bin/bash -e

# This is a convenience script to use the VM *without* Vagrant.
# Clones/updates the code from the Github repo.

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -d /vagrant ]]; then
  >&2 echo 'Only use this to update VMs running without Vagrant.'
  exit 1
fi

cd "${WORKSPACE_DIR}"

# Set up Gito repo.
if [[ ! -d ~/catkin_ws ]]; then
  eval "$(ssh-agent -s)"
  ./qtcopter_bootstrap/add_key.sh
  rm -f ~/.ssh/known_hosts
  ./qtcopter_bootstrap/connect_github.sh || true
  git clone ssh://git@github.com/fselius/qtcopter ~/catkin_ws

  # Replace this script with the one from the repo.
  ln -nfs ~/catkin_ws/update_qtcopter.sh "${WORKSPACE_DIR}/update_qtcopter.sh"
  rm -rf "${WORKSPACE_DIR}/qtcopter_bootstrap"
fi

cd ~/catkin_ws

if [[ -z "$(git config --local user.email)" \
   || -z "$(git config --local user.name)" ]]; then
  git config --local user.name "Qtcopter Bot"
  git config --local user.email "florian.walch@tum.de"
fi

# Update contents of Git repo.
eval "$(ssh-agent -s)"
./ssh/add_key.sh
git fetch origin
git stash save "autosave-$(date +%Y%m%d_%H%M%S)"
git checkout master
git reset --hard origin/master

# Install software etc.
./bootstrap.sh
