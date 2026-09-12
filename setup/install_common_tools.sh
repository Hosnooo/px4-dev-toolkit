#!/usr/bin/env bash
set -euo pipefail

echo "Installing common environment tools..."

sudo apt update
sudo apt install -y \
    tmux \
    python3-vcs2l

echo "Common environment tools ready."
