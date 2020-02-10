import sys              # argv
from math import sqrt

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} file1 [file2]'.format(sys.argv[0]))
    print('    Read two 4x4 matrices from two different files')
    print('    Return the sum of squares of the coefficient-wise differences')
    print('    The matrix are given in csv format:')
    print('        columns are separated by comma (,)')
    print('        rows are separated by linebreaks')
    print('    If file2 is left unspecified, the second matrix is read from stdin instead')
    print()
    sys.exit()

def get_files_desc():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help'] or len(sys.argv) < 1):
        print_usage_and_exit()
    else:
        if (len(sys.argv) > 1):
            fd1 = open(sys.argv[1])
            if (len(sys.argv) > 2):
                fd2 = open(sys.argv[2])
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

def main():
    fd1, fd2 = get_files_desc()
    diff = read_norm_of_diff(fd1,fd2)
    print(diff)
    close_files_desc(fd1,fd2)

if __name__=='__main__':
    main()
