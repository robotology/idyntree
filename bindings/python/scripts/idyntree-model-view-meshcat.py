#!/usr/bin/env python3

# SPDX-FileCopyrightText: Fondazione Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

import argparse

from idyntree.visualize import MeshcatVisualizer


def main():
    parser = argparse.ArgumentParser(
        description="Display the model in the meshcat visualizer."
    )
    parser.add_argument("--model", "-m", type=str, required=True, help="Model path.")
    parser.add_argument("--alpha", "-a", type=float, required=False, help="Alpha channel of the model.")

    args = parser.parse_args()

    # Load the visualizer
    viz = MeshcatVisualizer()
    viz.load_model_from_file(args.model, color=args.alpha)
    viz.open()

    run = True
    while run:
        quit = input("Enter q or Q to Quit: ")
        run = (quit != "q") and (quit != "Q")


if __name__ == "__main__":
    main()
