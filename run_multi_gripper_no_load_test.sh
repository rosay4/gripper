#!/usr/bin/env bash
set -e

# -------- configuration --------
ENV_NAME="gripper_test"
SCRIPT_NAME="beta1.0/hardware_guides/multi_gripper_no_load_test.py"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "========================================"
echo " Starting Multi Gripper No-load Test"
echo "========================================"
echo

# -------- locate conda base --------
if [[ -z "$CONDA_EXE" ]]; then
    if [[ -d "$HOME/miniconda3" ]]; then
        CONDA_BASE="$HOME/miniconda3"
    elif [[ -d "$HOME/anaconda3" ]]; then
        CONDA_BASE="$HOME/anaconda3"
    else
        echo "Cannot find conda installation"
        exit 1
    fi
else
    CONDA_BASE="$(dirname "$(dirname "$CONDA_EXE")")"
fi

ENV_PYTHON="$CONDA_BASE/envs/$ENV_NAME/bin/python"

if [[ ! -x "$ENV_PYTHON" ]]; then
    echo "Conda environment not found or broken:"
    echo "   $ENV_PYTHON"
    echo "   Please run env/install.sh first"
    exit 1
fi

echo "Using Python:"
echo "  $ENV_PYTHON"
echo

# -------- run script --------
cd "$SCRIPT_DIR"

if [[ ! -f "$SCRIPT_NAME" ]]; then
    echo "Script not found: $SCRIPT_NAME"
    exit 1
fi

echo "Running: $SCRIPT_NAME"
echo

exec "$ENV_PYTHON" "$SCRIPT_NAME" "$@"
