#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys
import numpy as np

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

def set_axes_equal(ax):
    #ax.set_aspect('equal')
    x_lim, y_lim, z_lim = ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()
    radius = 0.5 * max(abs(lim[1]-lim[0]) for lim in (x_lim, y_lim, z_lim))
    x_mid, y_mid, z_mid = (sum(lim)/2 for lim in (x_lim, y_lim, z_lim))
    ax.set_xlim3d([x_mid - radius, x_mid + radius])
    ax.set_ylim3d([y_mid - radius, y_mid + radius])
    ax.set_zlim3d([z_mid - radius, z_mid + radius])

def draw_pointcloud(pc):
    X,Y,Z,L = zip(*pc)
    #cmap = plt.cm.get_cmap('hsv', len(set(L)))
    #cmap = mcolors.ListedColormap(mcolors.TABLEAU_COLORS.values())
    vals = np.linspace(0,1,len(L))
    np.random.seed(987654321)
    np.random.shuffle(vals)
    cmap = plt.cm.colors.ListedColormap(plt.cm.jet(vals))
    fig = plt.figure()
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.scatter(X, Y, Z, s=0.4, c=L, cmap=cmap)
    set_axes_equal(ax)
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    ax2.set_xlabel('x')
    ax2.set_ylabel('y')
    ax2.set_zlabel('z')
    ax2.scatter(X, Y, Z, s=0.4, c=L, cmap=cmap)
    ax2.view_init(elev=330, azim = 180)
    set_axes_equal(ax2)

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} <pc file> [<img file>]'.format(cmd))
    print('      Display labeled pointcloud with one colour per label.')
    print('      Store resulting image to <img file>; or to "Figure1.png".')

def main(argv):
    if len(argv) > 1:
        pc_filename = argv[1]
        img_filename = argv[2] if len(argv) > 2 else 'Figure1.png'
    else:
        print_usage(argv[0])
    pc = read_pointcloud(pc_filename)
    draw_pointcloud(pc)
    #ax.view_axis(...)
    plt.savefig(img_filename)
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
