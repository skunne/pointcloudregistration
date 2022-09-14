#!/usr/bin/env python3

import sys
import numpy as np

def compute_rmse(pc_src, pc_dst, transform=None):
    if transform is None:
        transform = np.eye(4)
    (n_points,_),(n_points_dst,_) = pc_src.shape, pc_dst.shape
    assert(n_points == n_points_dst)
    assert(n_points > 0)
    pc_src_homo = np.c_[ pc_src, np.ones(n_points) ]
    pc_dst_homo = np.c_[ pc_dst, np.ones(n_points) ]
    pc_src_homo_transformed = pc_src_homo @ transform.T
    rmse = np.sqrt(((pc_dst_homo - pc_src_homo_transformed)**2).sum() / n_points)
    return rmse

def print_usage_and_exit(prog):
    print('SYNOPSIS')
    print()
    print('{} [transform] csvfile1 csvfile2'.format(prog))
    print('      Calculate the RMSE (root mean square error) between points in <csvfile1> and <csvfile2>.')
    print('      The two files must have the same number of points. The points must be')
    print('    listed in order. Point on line i is compared with point on line i.')
    print('      If a transform is specified, it must be in the form of a csv file with')
    print('    4 rows and 4 columns, representing the transformation matrix in homogeneous')
    print('    coordinates: a 3x3 linear transform, a 3x1 column translation vector, and')
    print('    a fourth line equal to 0,0,0,1.')
    sys.exit(1)

def get_args(argv):
    if len(argv) not in (3,4) or argv[1] in ('-h', '--help'):
        print_usage_and_exit(argv[0])
    elif len(argv) == 3:
        src_filename, dst_filename = argv[1], argv[2]
        transform_matrix_filename = None
    elif len(argv) == 4:
        src_filename, dst_filename = argv[2], argv[3]
        transform_matrix_filename = argv[1]
    return src_filename, dst_filename, transform_matrix_filename

def main(argv):
    src_filename, dst_filename, transform_matrix_filename = get_args(argv)
    pc_src = np.loadtxt(src_filename, delimiter=',', skiprows=1, usecols=(1,2,3))
    pc_dst = np.loadtxt(dst_filename, delimiter=',', skiprows=1, usecols=(1,2,3))
    transform_matrix =  np.loadtxt(transform_matrix_filename, delimiter=',') if transform_matrix_filename else None
    rmse = compute_rmse(pc_src, pc_dst, transform_matrix)
    print('{} {} {} --> RMSE = {}'.format(transform_matrix_filename, src_filename, dst_filename, rmse))

if __name__=='__main__':
    main(sys.argv)
