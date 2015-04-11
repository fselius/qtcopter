#!/usr/bin/env bash
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "${WORKSPACE_DIR}"
eval "$(ssh-agent -s)"
./ssh/add_key.sh
git fetch origin
git stash save "autosave-$(date +%Y%m%d_%H%M%S)"
git checkout master
git reset --hard origin/master

./bootstrap.sh
