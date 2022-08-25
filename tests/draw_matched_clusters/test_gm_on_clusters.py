#!/usr/bin/env/ python3

import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import sys
import numpy as np   # loadtxt
import math # hypot, isclose

def read_pointcloud(filename):
    pc = np.loadtxt(filename, delimiter=',', skiprows=1)
    #L, X, Y, Z = array[:,0], array[:,1], array[:,2], array[:,3]
    return pc

def read_perm_matrix(filename):
    mat = np.loadtxt(filename, int)
    return mat

def read_transform_matrix(filename):
    mat = np.loadtxt(filename,delimiter=',')
    return mat

def is_permutation_matrix(x):
    x = np.asanyarray(x)
    return (x.ndim == 2 and
            (x.sum(axis=0) <= 1).all() and
            (x.sum(axis=1) <= 1).all() and
            ((x == 1) | (x == 0)).all())

def build_permutation_dict(matrix):
    height,width = matrix.shape
    n = min(height, width)
    r_src = np.arange(height)
    r_dst = (r_src+1) @ matrix - 1
    return {j: i for i,j in enumerate(r_dst)}  #dict(zip(r_src, r_dst))

def isclose3d(p, q, rel_tol=10e-9):
    length = max(math.hypot(*p), math.hypot(*q))
    abs_tol = rel_tol * length
    return all(math.isclose(x1, x2, rel_tol=0, abs_tol=abs_tol) for x1,x2 in zip(p,q))

def rate_points(pc_src, pc_dst, perm_dict, transform):
    green, red = [], []
    n_points_notfound = 0
    target = {}
    for l,x,y,z in pc_dst:
        target.setdefault(l, set()).add((x,y,z))
    # print('Cluster 2 dans dst:')
    # print('    ', target[2])
    #pc_transformed = transform @ (pc_src add column 1)
    for (l_src,xs,ys,zs) in pc_src:
        l_dst = perm_dict.get(l_src, -1)
        if l_dst != -1:
            #if (x,y,z) in target[l_dst]:   # TODO apply transform and check for rounding errors
            p = (transform @ [xs,ys,zs, 1])[:-1] # transform matrix is written in homogeneous coordinates
            if any(isclose3d(p, q) for q in target[l_dst]):
                green.append((xs,ys,zs))
            else:
                red.append((xs,ys,zs))
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
    for ax_num in (1,2):
        ax = fig.add_subplot(1,2,ax_num,projection='3d')
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        #for pc, colour in zip((green, orange, red), ('green', 'orange', 'red')):
        for pc, colour in zip((green, red), ('green', 'red')):
            #X,Y,Z = zip(*pc)
            X = [x for x,_,_ in pc]
            Y = [y for _,y,_ in pc]
            Z = [z for _,_,z in pc]
            ax.scatter(X, Y, Z, s=0.4, c=colour)
        ax.view_init(elev = 390 - 30 * ax_num, azim = 180 * (ax_num-1))

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
        permutation_matrix_filename = argv[3]
        transform_filename = None
        output_filename = 'out.png'
        if len(argv) >= 5:
            transform_filename = argv[4]
            if len(argv) >= 6:
                output_filename = argv[5]
    else:
        print_usage(argv[0])
        sys.exit(-1)
    pc_src = read_pointcloud(pc_src_filename)
    pc_dst = read_pointcloud(pc_dst_filename)
    #pc_src, pc_dst = pc_dst, pc_src  # check if mistake dst src
    permutation_matrix = read_perm_matrix(permutation_matrix_filename)
    transform = read_transform_matrix(transform_filename) if transform_filename else np.eye(4)
    if is_permutation_matrix(permutation_matrix):
        print('Matrix is a correct permutation matrix')
    else:
        print('Matrix is not a correct permutation matrix!!')
        print(permutation_matrix)
    print('Transformation used:')
    print(transform)
    perm_dict = build_permutation_dict(permutation_matrix)
    green, orange, red = rate_points(pc_src, pc_dst, perm_dict, transform)
    draw_pointcloud(green, orange, red)
    plt.savefig(output_filename)
    plt.show()

if __name__ == '__main__':
    main(sys.argv)
