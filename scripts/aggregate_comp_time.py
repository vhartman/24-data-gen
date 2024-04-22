import numpy as np
import matplotlib.pyplot as plt

import os
import argparse

# plt.style.use('./scripts/paper_2.mplstyle')

def load_comp_times_from_txt_file(filename):
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
    
    return {
        'labels': labels,

        'totals': total_times,
        'rrt-times': rrt_times,
        'rrt-plan-times': rrt_plan_times,
        'coll-times': coll_times,
        'nn-times': nn_times,
        'shortcut-times': shortcut_times,
        'smoothing-times': smoothing_times,
        'komo-times': komo_times
    }

def load_folder(folder, time_offset = 0, iter_offset = 0):
    subfolders = [ f.path for f in os.scandir(folder) if f.is_dir() ]

    d = []
    for f in subfolders:
        if os.path.exists(f + "/computation_times.txt"):
            data = load_comp_times_from_txt_file(f + "/computation_times.txt")

            d.append(data)

    return d

def aggregate(data):
    agg = {}

    for tmp in data:
        for k, v in tmp.items():
            if k == 'labels':
                continue

            if k in agg:
                agg[k].extend(v)
            else:
                agg[k] = v

    return agg

def visualize_times(foldername):
    data = load_folder(foldername)
    agg_data = aggregate(data)

    sums = {}
    for k, v in agg_data.items():
        sums[k] = np.sum(v)
    
    plt.figure('percentage')
    plt.bar([l for l, _ in sums.items()], [v / sums['totals'] for _, v in sums.items()])

    plt.figure('sums')
    plt.bar([l for l, _ in sums.items()], [v for _, v in sums.items()])

    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', type=str, nargs='?', help='The file we want to have a look at.')

    args = parser.parse_args()
    folder = args.folder

    if folder is None:
        print("No folder specified")

    visualize_times(folder)

if __name__ == "__main__":
    main()
