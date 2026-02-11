#!/bin/bash

# 1. Kill any existing session to start fresh
tmux kill-session -t twin 2>/dev/null

# 2. Start a new session in the background
# -d: detached, -s: session name, -n: initial window name
tmux new-session -d -s twin -n "Servers"

# 3. Wait a split second for tmux to initialize
sleep 0.5

# 4. Pane 1: Mesh Server (Left Pane)
# We target the session directly to avoid index errors
tmux send-keys -t twin "cd ~/demo/snap-twin/test-base && python3 mesh_server.py" C-m

# 5. Split and Pane 2: Viz Server (Right Pane)
tmux split-window -h -t twin
tmux send-keys -t twin "conda activate snap-twin && cd ~/demo/snap-twin/test-base && python3 viz_server.py" C-m

# 6. Attach to the session
tmux attach-session -t twin
