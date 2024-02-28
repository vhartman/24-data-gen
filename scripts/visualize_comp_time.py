import numpy as np
import matplotlib.pyplot as plt

import os
import argparse

plt.style.use('./scripts/paper_2.mplstyle')

def visualize_times(filename):
    tmp = open(filename, "r").read()
    lines = tmp.split('\n')[:-1]

    total_times = []
    komo_times = []
    rrt_times = []
    rrt_plan_times = []
    coll_times = []
    nn_times = []
    shortcut_times = []
    smoothing_times = []

    labels = []

    for line in lines:
        robot = line.split(': ')[0]
        tasks = line.split(': ')[1].split('; ')[:-1]

        for task in tasks:
            total_time = float(task.split(', ')[-8])/1e6
            rrt_time =  float(task.split(', ')[-7])/1e6
            rrt_plan_time =  float(task.split(', ')[-6])/1e6
            coll_time = float(task.split(', ')[-5])/1e6
            nn_time = float(task.split(', ')[-4])/1e6
            smoothing_time = float(task.split(', ')[-3])/1e6
            shortcut_time = float(task.split(', ')[-2])/1e6
            komo_time = float(task.split(', ')[-1])/1e6
            
            labels.append(robot+task.split(', ')[-9])

            total_times.append(total_time)

            rrt_times.append(rrt_time)
            rrt_plan_times.append(rrt_plan_time)
            coll_times.append(coll_time)
            nn_times.append(nn_time)
            shortcut_times.append(shortcut_time)
            smoothing_times.append(smoothing_time)
            komo_times.append(komo_time)

    plt.xticks(np.arange(len(total_times)), labels, rotation=45)

    plt.plot(total_times, label='total')
    plt.plot(rrt_times, label='rrt')
    plt.plot(rrt_plan_times, label='rrt-plan')
    plt.plot(komo_times, label='komo')

    plt.plot(nn_times, label='nn')
    plt.plot(coll_times, label='coll')
    plt.plot(shortcut_times, label='shortcut')
    plt.plot(smoothing_times, label='smoothing')
    plt.legend()
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', type=str, nargs='?', help='The file we want to have a look at.')

    args = parser.parse_args()
    filename = args.file

    if filename is None:
        filename = "/home/valentin/git/personal-projects/23-shortcutting/out/Handover_1_20231127_183015"

    visualize_times(filename)

if __name__ == "__main__":
    main()
