#!/usr/bin/env python3

import sys              # argv
from math import sqrt

def print_usage_and_exit(cmd):
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(cmd))
    print('    print this help message and exit')
    print()
    print('{} file1 [file2]'.format(cmd))
    print('    Read two 4x4 matrices from two different files')
    print('    Return the sum of squares of the coefficient-wise differences')
    print('    The matrix are given in csv format:')
    print('        columns are separated by comma (,)')
    print('        rows are separated by linebreaks')
    print('    If file2 is left unspecified, the second matrix is read from stdin instead')
    print()
    sys.exit()

def get_files_desc(argv):
    if (len(argv) > 1 and argv[1] in ['-h', '--help'] or len(argv) < 1):
        print_usage_and_exit(cmd)
    else:
        if (len(argv) > 1):
            fd1 = open(argv[1])
            if (len(argv) > 2):
                fd2 = open(argv[2])
            else:
                fd2 = sys.stdin
    return fd1,fd2

def read_norm_of_diff(fd1, fd2):
    result = 0
    for line1, line2 in zip(fd1, fd2):
        row1, row2 = line1.split(','), line2.split(',')
        result += sum((float(a)-float(b))**2 for a,b in zip(row1, row2))
    return sqrt(result)

def close_files_desc(fd1,fd2):
    fd1.close()
    if fd2 is not sys.stdin:
        fd2.close()

def main(argv):
    fd1, fd2 = get_files_desc(argv)
    diff = read_norm_of_diff(fd1,fd2)
    print(diff)
    close_files_desc(fd1,fd2)

if __name__=='__main__':
    main(sys.argv)
