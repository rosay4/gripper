#!/usr/bin/env python3
import argparse
import os
import sys

cur_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(cur_dir)
folder_utils = os.path.join(project_root, "utils")
if folder_utils not in sys.path:
    sys.path.insert(0, folder_utils)

from visualize import draw


def parse_args():
    parser = argparse.ArgumentParser(description="Plot gripper low/high frequency logs.")
    parser.add_argument("--log-dir", required=True)
    parser.add_argument("--lowfile", required=True)
    parser.add_argument("--highfile", required=True)
    parser.add_argument("--savefig", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    draw(
        log_dir=args.log_dir,
        lowfile=args.lowfile,
        highfile=args.highfile,
        savefig=args.savefig,
        show_plot=False,
    )


if __name__ == "__main__":
    main()
