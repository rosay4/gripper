#!/usr/bin/env bash
set -e

# -------- configuration --------
ENV_NAME="gripper_test"
PLOT_ENV_NAME="pyqt6_env"
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
PLOT_ENV_PYTHON="$CONDA_BASE/envs/$PLOT_ENV_NAME/bin/python"

if [[ ! -x "$ENV_PYTHON" ]]; then
    echo "Conda environment not found or broken:"
    echo "   $ENV_PYTHON"
    echo "   Please run env/install.sh first"
    exit 1
fi

echo "Using Python:"
echo "  $ENV_PYTHON"
if [[ -x "$PLOT_ENV_PYTHON" ]]; then
    if MPLBACKEND=Agg "$PLOT_ENV_PYTHON" -c "import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot" >/dev/null 2>&1; then
        echo "Using plot Python:"
        echo "  $PLOT_ENV_PYTHON"
        PLOT_ARGS=(--plot-python "$PLOT_ENV_PYTHON")
    else
        echo "Plot environment exists but matplotlib is missing:"
        echo "  $PLOT_ENV_PYTHON"
        echo "  Install it with: conda activate $PLOT_ENV_NAME && conda install matplotlib"
        PLOT_ARGS=(--no-plot)
    fi
else
    echo "Plot environment not found, script will try fallback lookup:"
    echo "  $PLOT_ENV_PYTHON"
    PLOT_ARGS=()
fi
echo

# -------- run script --------
cd "$SCRIPT_DIR"

if [[ ! -f "$SCRIPT_NAME" ]]; then
    echo "Script not found: $SCRIPT_NAME"
    exit 1
fi

echo "Running: $SCRIPT_NAME"
echo

exec "$ENV_PYTHON" "$SCRIPT_NAME" "${PLOT_ARGS[@]}" "$@"
