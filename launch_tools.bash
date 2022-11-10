#!/bin/bash
WINDOW_ID=0
SESSION=""

function new_session() {
    SESSION=$1
    # # Create new session  (-2 allows 256 colors in the terminal, -s -> session name, -d -> not attach to the new session)
    # tmux -2 new-session -d -s $SESSION
}

function new_window() {
  if [ $WINDOW_ID -eq 0 ]; then
    # Kill any previous session (-t -> target session, -a -> all other sessions )
    tmux kill-session -t $SESSION
    # Create new session  (-2 allows 256 colors in the terminal, -s -> session name, -d -> not attach to the new session)
    tmux -2 new-session -d -s $SESSION

    # send-keys writes the string into the sesssion (-t -> target session , C-m -> press Enter Button)
    tmux rename-window -t $SESSION:0 "$1"
    tmux send-keys -t $SESSION:0 "$2" C-m

  else
    tmux new-window -t $SESSION:$WINDOW_ID -n "$1"
    tmux send-keys -t $SESSION:$WINDOW_ID "$2" C-m
  fi
  WINDOW_ID=$((WINDOW_ID+1))
}

function send_ctrl_c_tmux_session() {
    session_name=$1

    if [ -z "$session_name" ]; then
        echo "Usage: $0 <session_name>"
        exit 1
    fi

    windows=$(tmux list-windows -t "$session_name" | awk '{print $1}'| tr -d ':')
    for window in $windows; do
        # tmux kill-window -t "$session_name:$window"
        echo "Killed window $window"
        tmux send-keys -t "$session_name:$window" C-c
        tmux send-keys -t "$session_name:$window" "echo 'Killed window $window'" C-m
        # tmux send-keys -t "$session_name:$window" C-c
    done
    n_windows=$(tmux list-windows -t "$session_name" | wc -l)
    sleep "$n_windows"
    tmux kill-session -t "$session_name"
}
