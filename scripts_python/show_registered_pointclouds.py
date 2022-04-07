#!/usr/bin/env python3

import csv
import sys
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

identity_matrix = ((1.0,0,0,0),(0,1.0,0,0),(0,0,1.0,0),(0,0,0,1.0))

def get_matrix(filename):
    with open(filename, 'r') as f:
        r = csv.reader(f)
        m = [[float(x) for x in row] for row in r if row]
    return m

def matrix_times_vector(m, x):
    return [sum(m_ij * x_j for m_ij, x_j in zip(m_i, x)) for m_i in m]

def matrix_times_pointcloud(m, X,Y,Z):
    new_pc = [matrix_times_vector(m, (x,y,z,1))[:-1] for x,y,z in zip(X,Y,Z)]
    new_X,new_Y,new_Z = [list(axis) for axis in zip(*new_pc)]
    return new_X,new_Y,new_Z

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

def print_list(ll):
    for x in ll[:-1]:
        print(x, end=',')
    print(ll[-1])

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} [<transform>] <pcdfile1> <pcdfile2>'.format(cmd))
    print('      Calculate the average distance between points in <pcdfile1> and <pcdfile2>.')
    print('      The two files must have the same number of points. The points must be')
    print('    listed in order. Point on line i is compared with point on line i.')
    print('      If a transform is specified, it must be in the form of a csv file with')
    print('    4 rows and 4 columns, representing the transformation matrix in homogeneous')
    print('    coordinates: a 3x3 linear transform, a 3x1 column translation vector, and')
    print('    a fourth line equal to 0,0,0,1.')
    sys.exit(1)

def main(argv):
    if len(argv) == 4:
        matrix, file1, file2 = get_matrix(argv[1]), argv[2], argv[3]
    elif len(argv) == 3:
        print('Using identity transform')
        matrix, file1, file2 = identity_matrix, argv[1], argv[2]
    else:
        print_usage(argv[0])
        exit()
    #list_of_d = compute_average_distance(file1, file2, matrix)
    print(matrix)
    pc_src = read_file(file1)
    pc_dst = read_file(file2)
    if len(argv) == 4:
        pc_src_transformed = matrix_times_pointcloud(matrix, *pc_src)
    else:
        pc_src_transformed = pc_src
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(*pc_src_transformed, s=0.7)
    ax.scatter(*pc_dst, s=0.7)
    plt.show()

if __name__=='__main__':
    main(sys.argv)
