#!/usr/bin/env python3

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import sys

def read_file(filename):
    print('Opening point cloud file {}'.format(filename))
    with open(filename, 'r') as f:
      X = []
      Y = []
      Z = []
      for line in f:
        row = line.split()
        if row[0][0] in '0123456789-':
          X.append(float(row[0]))
          Y.append(float(row[1]))
          Z.append(float(row[2]))
    print('nb points: {}'.format(len(X)))
    return X,Y,Z

def plotplot(X,Y,Z):
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    # ax.set_xlim(left=-10, right=10)
    # ax.set_ylim(bottom=-10, top=10)
    # ax.set_zlim(bottom=-10, top=10)
    #ax.axis('equal')
    ax.scatter(X,Y,Z, s=0.7)
    return fig,ax

def main():
    X,Y,Z = read_file(sys.argv[1])
    fig,ax = plotplot(X,Y,Z)
    if len(sys.argv) > 2:
        print('Saving plot to {}'.format(sys.argv[2]))
        fig.savefig(sys.argv[2])
    fig.show()

if __name__=='__main__':
    main()
