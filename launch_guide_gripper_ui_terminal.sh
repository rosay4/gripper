#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUN_CMD="cd \"$SCRIPT_DIR\" && bash run_guide_gripper_ui.sh"

if [[ "${GRIPPER_LAUNCHED_TERMINAL:-0}" == "1" ]]; then
    cd "$SCRIPT_DIR"
    bash run_guide_gripper_ui.sh
    status=$?
    echo
    echo "GripperTest exited with status: $status"
    echo "$RUN_CMD" >> "$HOME/.bash_history" 2>/dev/null || true
    echo "Terminal is kept open. Press Up to recall the launch command, or type exit to close."
    exec bash -i
fi

if command -v gnome-terminal >/dev/null 2>&1; then
    exec gnome-terminal --full-screen -- bash -lc "export GRIPPER_LAUNCHED_TERMINAL=1; $RUN_CMD"
elif command -v x-terminal-emulator >/dev/null 2>&1; then
    exec x-terminal-emulator -e bash -lc "export GRIPPER_LAUNCHED_TERMINAL=1; $RUN_CMD"
else
    export GRIPPER_LAUNCHED_TERMINAL=1
    exec bash -lc "$RUN_CMD"
fi
