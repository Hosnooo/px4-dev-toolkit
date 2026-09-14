#!/usr/bin/env bash
set -euo pipefail

# Small host-side toolset used directly by this repository's scripts.
echo "Installing common environment tools..."

sudo apt update
sudo apt install -y \
    tmux \
    python3-venv \
    python3-numpy python3-matplotlib python3-scipy \
    python3-vcs2l

echo "Common environment tools ready."
