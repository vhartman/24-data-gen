import numpy as np
import matplotlib.pyplot as plt

import os

plt.style.use('./scripts/paper_2.mplstyle')

def visualize_times(filename):
    tmp = open(filename, "r").read()
    lines = tmp.split('\n')[:-1]

    komo_times = []
    rrt_times = []

    for line in lines:
        robot = line.split(': ')[0]
        tasks = line.split(': ')[1].split('; ')[:-1]

        for task in tasks:
            total_time = float(task.split(', ')[-3])/1e6
            rrt_time =  float(task.split(', ')[-2])/1e6
            komo_time = float(task.split(', ')[-1])/1e6

            rrt_times.append(rrt_time)
            komo_times.append(komo_time)

    plt.plot(rrt_times, label='rrt')
    plt.semilogy(komo_times, label='komo')
    plt.legend()
    plt.show()

def main():
    filename = "/home/valentin/git/personal-projects/23-sim-an/out/greedy_20230921_142837/1/computation_times.txt"
    filename = "/home/valentin/git/personal-projects/23-sim-an/out/greedy_20230921_144350/3/computation_times.txt"
    filename = "/home/valentin/git/personal-projects/23-sim-an/out/greedy_20230921_145446/1/computation_times.txt"
    visualize_times(filename)

if __name__ == "__main__":
    main()
