import numpy as np
import matplotlib.pyplot as plt

import os

plt.style.use('./scripts/paper_2.mplstyle')

def read_integer_from_file(filename):
    tmp = open(filename, "r")
    return int(tmp.read())

def load_folder(folder, time_offset = 0, iter_offset = 0):
    subfolders = [ f.path for f in os.scandir(folder) if f.is_dir() ]

    d = []
    for f in subfolders:
        if f.split('/')[-1] != "info":
            if os.path.exists(f + "/makespan.txt"):
                makespan = read_integer_from_file(f + "/makespan.txt")
                time = read_integer_from_file(f + "/comptime.txt") + time_offset
                iteration = int(f.split('/')[-1]) + iter_offset

                d.append((time, makespan, iteration))

    d = sorted(d)

    return d

def plot_folder(ax, foldername, color='tab:blue', single_point_length=None):
    if type(foldername) == list:
        data = []
        time_offset = 0
        iter_offset = 0
        for folder in foldername:
            single_folder_data = load_folder(folder, time_offset, iter_offset)
            data.extend(single_folder_data)
            time_offset = data[-1][0]
            iter_offset = data[-1][2]
    else:
        data = load_folder(foldername)

    x = [d[0] for d in data]
    y = [d[1] for d in data]
    iters = [d[2] for d in data]

    #for i, _ in enumerate(iters):
    #    if iters[i] > 69:
    #        y[i] = y[i]-120

    overall_min = [y[0]]
    for val in y[1:]:
        overall_min.append(min([val, overall_min[-1]]))

    #plt.plot(y, drawstyle="steps", color='tab:blue')
    #plt.plot(overall_min, ls='--', drawstyle="steps", color='tab:blue')
    
    #x = iters
    x = np.array(x) / 60 / 60

    ax.plot(x, y, ls='--', drawstyle="steps-post", color=color)
    ax.plot(x, overall_min, ls='-', drawstyle="steps-post", color=color)

    if single_point_length is not None:
        ax.plot([0, single_point_length / 60 / 60], [overall_min[-1], overall_min[-1]], ls='-', drawstyle="steps-post", color=color)

def plot_grid_exp():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # grid experiments
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_000824/"
    greedy_folder = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_095747/"
    single_arm = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid/greedy_20230328_141909/"
    plot_folder(ax, foldername)
    plot_folder(ax, greedy_folder, "tab:orange", 33800)
    plot_folder(ax, single_arm, "tab:green", 33800)

    plt.xlabel('Computation time [h]')
    plt.ylabel('Makespan')

    plt.grid(which='major', axis='y', ls='--')
    tmp = foldername.split("/")[-2]
    plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

def plot_bin_pick_exp():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # grid experiments
    #foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230328_211219/"
    single_arm = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_230346/"
    greedy_folder = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_225950/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230328_211219/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/bin/greedy_20230330_005636/"

    plot_folder(ax, foldername)
    plot_folder(ax, greedy_folder, "tab:orange", 4.08*3600)
    plot_folder(ax, single_arm, "tab:green", 4.09*3600)

    plt.xlabel('Computation time [h]')
    plt.ylabel('Makespan')

    plt.grid(which='major', axis='y', ls='--')
    tmp = foldername.split("/")[-2]
    plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

def plot_lislarge_exp():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # grid experiments
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lislarge/greedy_20230330_110223/"
    greedy_folder = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lislarge/greedy_20230329_194144/"
    single_arm = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lislarge/greedy_20230329_192119/"
    plot_folder(ax, foldername)
    plot_folder(ax, greedy_folder, "tab:orange", 8.9*3600)
    plot_folder(ax, single_arm, "tab:green", 8.9*3600)

    plt.xlabel('Computation time [h]')
    plt.ylabel('Makespan')

    plt.grid(which='major', axis='y', ls='--')
    tmp = foldername.split("/")[-2]
    plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

def plot_lis_exp():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # grid experiments
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_000242/"
    greedy_folder = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_103317/"
    single_arm = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_104428/"
    plot_folder(ax, foldername)
    plot_folder(ax, greedy_folder, "tab:orange", 37900)
    plot_folder(ax, single_arm, "tab:green", 37900)

    plt.xlabel('Computation time [h]')
    plt.ylabel('Makespan')

    plt.grid(which='major', axis='y', ls='--')
    tmp = foldername.split("/")[-2]
    plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

def plot_four_arm_grid():
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)

    # grid experiments
    foldernames = [
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230330_221823/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_094537/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_020353/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230331_235555/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_165530/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_010207/",
        "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230401_010325/",
        #"/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/opt/greedy_20230331_102923/",
    ]
    greedy_folder = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/grid_four/greedy_20230331_180648/"
    #single_arm = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lis/greedy_20230329_104428/"
    plot_folder(ax, foldernames)
    plot_folder(ax, greedy_folder, "tab:orange", 37900)
    #plot_folder(ax, single_arm, "tab:green", 37900)

    plt.xlabel('Computation time [h]')
    plt.ylabel('Makespan')

    plt.grid(which='major', axis='y', ls='--')
    tmp = foldernames[0].split("/")[-2]
    plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

def plot_experiments():
    plot_grid_exp()
    plot_bin_pick_exp()
    plot_lis_exp()

def main():
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230324_181414/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230324_203952/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230325_001644/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230325_002536/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230327_180715/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230327_184654/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230327_200637/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230328_000824/"
    #data = load_folder(foldername)

    #for i, iteration in enumerate(iters):
    #    if (iteration-1)%20 == 0:
    #        plt.plot([x[i], x[i]], [250, 350], color='black', alpha=0.5, ls='--')

    # grid experiments
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230329_212744/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230329_213220/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lislarge/greedy_20230330_005814/"
    foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/exp/lislarge/greedy_20230330_110223/"
    #foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230330_005636/"
    #foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230330_110223/"
    if True:
        fig = plt.figure()
        ax = fig.add_subplot(1, 1, 1)

        foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230331_181739/"
        foldername = "/home/valentin/git/manipulation-planning/examples/23-sim-an/out/greedy_20230401_010325/"
        plot_folder(ax, foldername)

        plt.xlabel('Computation time [h]')
        plt.ylabel('Makespan')

        plt.grid(which='major', axis='y', ls='--')
        tmp = foldername.split("/")[-2]
        #plt.savefig(f'./out/plots/{tmp}.pdf', format='pdf', dpi=300, bbox_inches = 'tight')

    #plot_bin_pick_exp()
    #plot_experiments()
    #plot_lislarge_exp()
    plot_four_arm_grid()

    plt.show()

main()
