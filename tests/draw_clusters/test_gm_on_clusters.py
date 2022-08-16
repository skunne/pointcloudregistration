#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys

def read_pointcloud(filename):
    pc = []
    with open(filename, 'r') as f:
        for line in f:
            xyz, label = line.strip('()\n').split(' - ')
            x,y,z = xyz.split(',')
            try:
                x,y,z,label = map(int, (x,y,z,label))
            except ValueError:
                x,y,z = map(float, (x,y,z))
                label = int(label)
            pc.append((x,y,z,label))
    return pc

# def draw_pointcloud(pc):
#     X,Y,Z,L = zip(*pc)
#     #cmap = plt.cm.get_cmap('hsv', len(set(L)))
#     cmap = mcolors.ListedColormap(mcolors.TABLEAU_COLORS.values())
#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')
#     ax.set_xlabel('x')
#     ax.set_ylabel('y')
#     ax.set_zlabel('z')
#     ax.scatter(X, Y, Z, c=L, cmap=cmap)

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} <pc file 1> <pc file 2> [<>]'.format(cmd))
    print('      ...')
    print('      ...')

def main(argv):
    if len(argv) > 1:
        pc_src = argv[1]
    else:
        print_usage(argv[0])
    pc = read_pointcloud(pc_src)

if __name__ == '__main__':
    main(sys.argv)
