import csv
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
    d = 0
    with open(filename1, 'r') as f1:
        with open(filename2, 'r') as f2:
            r1 = csv.reader(f1)
            r2 = csv.reader(f2)
            next(r1)  # remove header
            next(r2)  # remove header
            nbpoints = 0
            for p1, p2 in zip(r1,r2):
                p1_homo = [float(xj) for xj in p1[1:]] + [1]
                p2_homo = [float(xj) for xj in p2[1:]] + [1]
                p1_homo_transformed = matrix_times_vector(transform, p1_homo)
                d += math.sqrt(sum((x1 - x2)**2 for (x1,x2) in zip(p1_homo_transformed[:-1], p2_homo[:-1])))
                nbpoints += 1
    if nbpoints > 0:
        return d / nbpoints
    else:
        return 0

def print_usage():
    print('SYNOPSIS')
    print()
    print('{} [transform] csvfile1 csvfile2'.format(sys.argv[0]))
    print('      Calculate the average distance between points in <csvfile1> and <csvfile2>.')
    print('      The two files must have the same number of points. The points must be')
    print('    listed in order. Point on line i is compared with point on line i.')
    print('      If a transform is specified, it must be in the form of a csv file with')
    print('    4 rows and 4 columns, representing the transformation matrix in homogeneous')
    print('    coordinates: a 3x3 linear transform, a 3x1 column translation vector, and')
    print('    a fourth line equal to 0,0,0,1.')
    sys.exit(1)

def main():
    if len(sys.argv) == 3:
        print(compute_average_distance(sys.argv[1], sys.argv[2]))
    elif (len(sys.argv) == 4):
        print(compute_average_distance(sys.argv[2], sys.argv[3], get_matrix(sys.argv[1])))
    else:
        print_usage()

if __name__=='__main__':
    main()
