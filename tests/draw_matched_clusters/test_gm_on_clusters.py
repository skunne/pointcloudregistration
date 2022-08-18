#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys
import numpy as np   # loadtxt

def read_pointcloud(filename):
    pc = np.loadtxt(filename, delimiter=',', skiprows=1)
    #L, X, Y, Z = array[:,0], array[:,1], array[:,2], array[:,3]
    return pc

def read_matrix(filename):
    mat = np.loadtxt(filename, int)
    return mat

def is_permutation_matrix(x):
    x = np.asanyarray(x)
    return (x.ndim == 2 and
            (x.sum(axis=0) <= 1).all() and
            (x.sum(axis=1) <= 1).all() and
            ((x == 1) | (x == 0)).all())

def build_permutation_dict(matrix):
    width, height = matrix.shape
    n = min(width, height)
    r_src = np.arange(width)
    r_dst = matrix @ r_src
    # print('permutation:')
    # print('    ', r_src)
    # print('    ', r_dst)
    #r_dst = r_dst.astype(int)
    return dict(zip(r_src, r_dst))

def rate_points(pc_src, pc_dst, perm_dict):
    green, red = [], []
    n_points_notfound = 0
    target = {}
    for l,x,y,z in pc_dst:
        target.setdefault(l, set()).add((x,y,z))
    # print('Cluster 2 dans dst:')
    # print('    ', target[2])
    for (l_src,x,y,z) in pc_src:
        l_dst = perm_dict.get(l_src, -1)
        if l_dst != -1:
            if (x,y,z) in target[l_dst]:   # TODO apply transform and check for rounding errors
                green.append((x,y,z))
            else:
                red.append((x,y,z))
        else:
            n_points_notfound += 1
    if n_points_notfound > 0:
        print('Number of points not found:')
        print('    {} / {}'.format(n_points_notfound, len(target)))
    orange = []
    print('Green points:  ', len(green))
    print('Orange points: ', len(orange))
    print('Red points:    ', len(red))
    return green, orange, red

# def rate_points(pc_src, pc_dst):
#     green, red = [], []
#     target = {(x,y,z): l for l,x,y,z in pc_dst}
#     n_points_notfound = 0
#     for (l,x,y,z) in pc_src:
#         if (x, y, z) in target:
#             if l == target.get((x, y, z), -1):
#                 green.append((x,y,z))
#             else:
#                 red.append((x,y,z))
#         else:
#             n_points_notfound += 1
#     if n_points_notfound > 0:
#         print('Number of points not found:')
#         print('    {} / {}'.format(n_points_notfound, len(target)))
#     return green, [], red

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
        #X,Y,Z = zip(*pc)
        X = [x for x,_,_ in pc]
        Y = [y for _,y,_ in pc]
        Z = [z for _,_,z in pc]
        ax.scatter(X, Y, Z, c=colour)

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} <pc file 1> <pc file 2> <matrix file> [<img file>]'.format(cmd))
    print('      ...')
    print('      output to img file')

def main(argv):
    if len(argv) >= 4:
        pc_src_filename = argv[1]
        pc_dst_filename = argv[2]
        matrix_filename = argv[3]
        if len(argv) >= 5:
            output_filename = argv[4]
        else:
            output_filename = 'out.png'
    else:
        print_usage(argv[0])
        sys.exit(-1)
    pc_src = read_pointcloud(pc_src_filename)
    pc_dst = read_pointcloud(pc_dst_filename)
    #pc_src, pc_dst = pc_dst, pc_src  # check if mistake dst src
    matrix = read_matrix(matrix_filename)
    if is_permutation_matrix(matrix):
        print('Matrix is a correct permutation matrix')
    else:
        print('Matrix is not a correct permutation matrix!!')
        print(matrix)
    perm_dict = build_permutation_dict(matrix)
    green, orange, red = rate_points(pc_src, pc_dst, perm_dict)
    draw_pointcloud(green, orange, red)
    plt.savefig(output_filename)
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
