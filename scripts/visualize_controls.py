import numpy as np
import matplotlib.pyplot as plt

import os
import argparse

plt.style.use('./scripts/paper_2.mplstyle')

def visualize_controls(filename):
    tmp = open(filename, "r").read().replace(']', '').replace('[', '')
    lines = tmp.split(',\n')
    lines = [l.split(', ') for l in lines]
    numbers = [[float(n) for n in line] for line in lines]

    numbers = np.array(numbers)
    for i in range(1):
        robot = numbers[:, 7*i:7*(i+1)]
        plt.figure("pos")
        plt.plot(robot)
        plt.xlabel("time")
        plt.ylabel("pos [rad]")

        plt.figure("vel")
        plt.plot(np.diff(robot, axis=0))
        plt.xlabel("time")
        plt.ylabel("vel [rad/s]")

        plt.figure("acc")
        plt.plot(np.diff(np.diff(robot, axis=0), axis=0))
        plt.xlabel("time")
        plt.ylabel("acc [rad/s2]")

    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, nargs='?', help='The file we want to have a look at.')

    args = parser.parse_args()
    controls_file = args.file

    if controls_file is None:
        controls_file = "/home/valentin/git/personal-projects/23-sim-an/out/greedy_20230921_142837/1/robot_controls.txt"
    
    visualize_controls(controls_file)

if __name__ == "__main__":
    main()
