#!/usr/bin/env python3


"""
Copyright (C) 2022 Fondazione Istituto Italiano di Tecnologia

Licensed under either the GNU Lesser General Public License v3.0 :
https://www.gnu.org/licenses/lgpl-3.0.html
or the GNU Lesser General Public License v2.1 :
https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
at your option.
"""

import argparse

from idyntree.visualize import MeshcatVisualizer


def main():
    parser = argparse.ArgumentParser(description='Display the model in the meshcat visualizer.')
    parser.add_argument('--model', '-m', type = str, required = True, help='Model path.')

    args = parser.parse_args()

    # Load the visualizer
    viz = MeshcatVisualizer()
    viz.set_model_from_file(args.model)
    viz.load_model()
    viz.open()

    run = True
    while run:
        quit = input('Enter q or Q to Quit: ')
        run = (quit != 'q') and (quit != 'Q')


if __name__ == "__main__":
    main()
