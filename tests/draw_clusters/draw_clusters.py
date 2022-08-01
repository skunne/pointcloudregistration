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

def main(argv):
    pc = read_pointcloud(argv[1])
    draw_pointcloud(pc)
    plt.savefig('Figure1.png')
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
