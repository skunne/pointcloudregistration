#! /usr/bin/env python3

import csv # used in get_matrix
import sys
import math

identity_matrix = ((1.0,0,0,0),(0,1.0,0,0),(0,0,1.0,0),(0,0,0,1.0))

def get_matrix(filename):
    with open(filename, 'r') as f:
        r = csv.reader(f)
        m = [[float(x) for x in row] for row in r]
    return m

def matrix_times_vector(m, x):
    return [sum(m_ij * x_j for m_ij, x_j in zip(m_i, x)) for m_i in m]

def compute_average_distance(filename1, filename2, transform=identity_matrix):
    headerfields = ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']
    d = 0
    with open(filename1, 'r') as f1:
        with open(filename2, 'r') as f2:
            nbpoints = 0
            for line1, line2 in zip(f1,f2):
                row1, row2 = line1.split(), line2.split()
                if row1[0] not in headerfields and row2[0] not in headerfields:
                    p1 = [float(xj) for xj in row1][:-1] + [1]
                    p2 = [float(xj) for xj in row2][:-1] + [1]
                    q1 = matrix_times_vector(transform, p1)
                    d += math.sqrt(sum((x1-x2)**2 for (x1,x2) in zip(q1[:-1], p2[:-1])))
                    nbpoints += 1
                elif row1[0] not in headerfields or row2[0] not in headerfields:
                    print('WARNING: mismatched header lines')
    if nbpoints > 0:
        return d / nbpoints
    else:
        return 0

def print_usage(cmd):
    print('SYNOPSIS')
    print()
    print('{} [transform] csvfile1 csvfile2'.format(cmd))
    print('      Calculate the average distance between points in <csvfile1> and <csvfile2>.')
    print('      The two files must have the same number of points. The points must be')
    print('    listed in order. Point on line i is compared with point on line i.')
    print('      If a transform is specified, it must be in the form of a csv file with')
    print('    4 rows and 4 columns, representing the transformation matrix in homogeneous')
    print('    coordinates: a 3x3 linear transform, a 3x1 column translation vector, and')
    print('    a fourth line equal to 0,0,0,1.')
    sys.exit(1)

def main(argv):
    if len(argv) == 3:
        print(compute_average_distance(argv[1], argv[2]))
    elif (len(argv) == 4):
        print(compute_average_distance(argv[2], argv[3], get_matrix(argv[1])))
    else:
        print_usage(argv[0])

if __name__=='__main__':
    main(sys.argv)
