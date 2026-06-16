#!/bin/bash
# Update repositories for Noble-Updates
sudo tee /etc/apt/sources.list.d/ubuntu.sources <<EOF
Types: deb
URIs: http://ports.ubuntu.com/ubuntu-ports
Suites: noble noble-updates noble-backports
Components: main restricted universe multiverse
Signed-By: /usr/share/keyrings/ubuntu-archive-keyring.gpg
EOF

sudo apt update && sudo apt upgrade -y
sudo apt install ros-jazzy-ros-base ros-dev-tools ros-jazzy-ros-gz \
                 ros-jazzy-foxglove-bridge ros-jazzy-gz-tools-vendor tmux -y
