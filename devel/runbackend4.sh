#!/bin/sh

tmux kill-session -t backend
tmux new-session -d -s backend 'runbackend.sh'
tmux rename-window 'roscore'
tmux select-window -t backend:0
tmux split-window -h 'runbackend.sh'
tmux split-window -v -t 0 'runbackend.sh'
tmux split-window -v -t 1 'runbackend.sh'
tmux -2 attach-session -t backend
