#!/usr/bin/env bash
set -e

# -------- configuration --------
ENV_NAME="gripper_test"
SCRIPT_NAME="beta1.0/hardware_guides/guide_gripper_ui.py"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CAN_INIT_SCRIPT="$SCRIPT_DIR/../env/init_socket_can.sh"

echo "========================================"
echo " Starting Gripper Guide UI"
echo "========================================"
echo

# -------- maximize terminal when possible --------
if [[ -t 1 ]]; then
    if [[ -n "${WINDOWID:-}" ]] && command -v xdotool >/dev/null 2>&1; then
        xdotool windowmove "$WINDOWID" 0 0 >/dev/null 2>&1 || true
        xdotool windowsize "$WINDOWID" 100% 100% >/dev/null 2>&1 || true
    elif command -v wmctrl >/dev/null 2>&1; then
        wmctrl -r :ACTIVE: -b add,maximized_vert,maximized_horz >/dev/null 2>&1 || true
    fi
    printf '\033[9;1t\033[8;40;120t'
    sleep 0.2
fi

# -------- locate conda base --------
if [[ -z "$CONDA_EXE" ]]; then
    if [[ -d "$HOME/miniforge3" ]]; then
        CONDA_BASE="$HOME/miniforge3"
    elif [[ -d "$HOME/miniconda3" ]]; then
        CONDA_BASE="$HOME/miniconda3"
    elif [[ -d "$HOME/anaconda3" ]]; then
        CONDA_BASE="$HOME/anaconda3"
    else
        echo "❌ Cannot find conda installation"
        exit 1
    fi
else
    CONDA_BASE="$(dirname "$(dirname "$CONDA_EXE")")"
fi

ENV_PYTHON="$CONDA_BASE/envs/$ENV_NAME/bin/python"

if [[ ! -x "$ENV_PYTHON" ]]; then
    echo "❌ Conda environment not found or broken:"
    echo "   $ENV_PYTHON"
    echo "   Please run install.sh first"
    exit 1
fi

echo "Using Python:"
echo "  $ENV_PYTHON"
echo

# -------- initialize SocketCAN --------
if [[ -f "$CAN_INIT_SCRIPT" ]]; then
    echo "Initializing SocketCAN:"
    echo "  $CAN_INIT_SCRIPT"
    if [[ "$EUID" -eq 0 ]]; then
        bash "$CAN_INIT_SCRIPT" || true
    elif command -v sudo >/dev/null; then
        sudo BITRATE="${CAN_BITRATE:-1000000}" TXQLEN="${CAN_TXQLEN:-65535}" bash "$CAN_INIT_SCRIPT" || true
    else
        echo "⚠️ sudo not found, skipping SocketCAN init"
    fi
    echo
fi

# -------- run script --------
cd "$SCRIPT_DIR"

if [[ ! -f "$SCRIPT_NAME" ]]; then
    echo "❌ Script not found: $SCRIPT_NAME"
    exit 1
fi

echo "Running: $SCRIPT_NAME"
echo

exec "$ENV_PYTHON" "$SCRIPT_NAME"
