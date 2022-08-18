#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys
import numpy as np   # loadtxt

def read_pointcloud(filename):
    pc = np.loadtxt(filename, delimiter=',', skiprows=1)
    #L, X, Y, Z = array[:,0], array[:,1], array[:,2], array[:,3]
    return pc

def rate_points(pc_src, pc_dst):
    green, red = [], []
    target = {(x,y,z): l for l,x,y,z in pc_dst}
    n_points_notfound = 0
    for (l,x,y,z) in pc_src:
        if (x+10, y+10, z+10) in target:
            if l == target.get((x, y, z), -1):
                green.append((x,y,z))
            else:
                red.append((x,y,z))
        else:
            n_points_notfound += 1
    if n_points_notfound > 0:
        print('Number of points not found:')
        print('    {} / {}'.format(n_points_notfound, len(target)))
    return green, [], red

def draw_pointcloud(green, orange, red):
    #cmap = plt.cm.get_cmap('hsv', len(set(L)))
    # cmap = mcolors.ListedColormap(mcolors.TABLEAU_COLORS.values())
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    #for pc, colour in zip((green, orange, red), ('green', 'orange', 'red')):
    for pc, colour in zip((green, red), ('green', 'red')):
        X,Y,Z = zip(*pc)
        ax.scatter(X, Y, Z, c=colour)

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} <pc file src> <pc file dst> [<img file>]'.format(cmd))
    print('      ...')
    print('      output to img file')

def main(argv):
    if len(argv) >= 3:
        pc_src_filename = argv[1]
        pc_dst_filename = argv[2]
        if len(argv) >= 4:
            output_filename = argv[3]
        else:
            output_filename = 'out.png'
    else:
        print_usage(argv[0])
        sys.exit(-1)
    pc_src = read_pointcloud(pc_src_filename)
    pc_dst = read_pointcloud(pc_dst_filename)
    green, orange, red = rate_points(pc_src, pc_dst)
    draw_pointcloud(green, orange, red)
    plt.savefig(output_filename)
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
