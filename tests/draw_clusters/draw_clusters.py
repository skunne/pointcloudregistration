#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import sys

def read_pointcloud(filename):
    pc = []
    with open(filename, 'r') as f:
        for line in f:
            xyz, label = line.strip('()\n').split(' - ')
            x,y,z = xyz.split(',')
            x,y,z,label = map(int, (x,y,z,label))
            pc.append((x,y,z,label))
    return pc

def draw_pointcloud(pc):
    X,Y,Z,L = zip(*pc)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(X, Y, Z, c=L)

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
    plt.savefig(img_filename)
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
