import csv
import sys
import math

def compute_average_distance(filename1, filename2):
    d = 0
    with open(filename1, 'r') as f1:
        with open(filename2, 'r') as f2:
            r1 = csv.reader(f1)
            r2 = csv.reader(f2)
            next(r1)  # remove header
            next(r2)  # remove header
            nbpoints = 0
            for p1, p2 in zip(r1,r2):
                d += math.sqrt(sum((float(x1) - float(x2))**2 for (x1,x2) in zip(p1[1:], p2[1:])))
                nbpoints += 1
    if nbpoints > 0:
        return d / nbpoints
    else:
        return 0

def print_usage():
    print('SYNOPSIS')
    print()
    print('{} csvfile1 csvfile2'.format(sys.argv[0]))
    print('    Calculate the average distance between the points in <csvfile1> and <csvfile2>.')
    print('    The two files must have the same number of points. The points must be listed in order. Point on line i is compared with point on line i.')
    sys.exit(1)

def main():
    if len(sys.argv) == 3:
        print(compute_average_distance(sys.argv[1], sys.argv[2]))
    else:
        print_usage()

if __name__=='__main__':
    main()
