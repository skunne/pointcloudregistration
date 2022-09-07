#!/usr/bin/env python3

import sys
from numpy import loadtxt, array # argv, exit()
from math import atan2, sqrt, pi, isclose
from fractions import Fraction

def print_usage_and_exit(cmd):
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(cmd))
    print('    print this help message and exit')
    print()
    print('{} <matrix1> <matrix2>'.format(cmd))
    print('    Compare two matrices')
    print('    The two matrices must be in comma-separated csv files')
    print('    In homogeneous coordinates: 4x4 matrices')
    print('        * The upperleft 3x3 quadrant is a rotation')
    print('        * The right column is a translation')
    print('        * The bottom row is 0,0,0,1')
    print('    Output: 6 numbers')
    print('        * The 3 coordinates of a translation vector')
    print('        * The three Euler angles of a rotation factorised as RzRyRx')
    print()
    sys.exit()

def get_args(argv):
    if (len(argv) > 1 and argv[1] in ['-h', '--help']):
        print_usage_and_exit(argv[0])
    elif (len(argv) != 3):
        print_usage_and_exit(argv[0])
    elif (len(argv) == 3):
        mat1_filename = argv[1]
        mat2_filename = argv[2]
        return (mat1_filename, mat2_filename)

def read_matrix(filename):
    m = loadtxt(filename, delimiter=',')
    return m

def euler_angles(m):
    if isclose(m[2,0], 1):
        print('Warning: Euler angle_y == -pi/2, no unique solution.')
        return (0, -pi / 2, atan2(-m[1,2] , m[1,1]))
    elif isclose(m[2,0], -1):
        print('Warning: Euler angle_y == pi/2, no unique solution.')
        return (0, pi / 2, -atan2(-m[1,2] , m[1,1]))
    else:
        return (atan2(m[2,1], m[2,2]),
                atan2(-m[2,0], sqrt(m[2,1]**2 + m[2,2]**2)),
                atan2(m[1,0], m[0,0]))

def rad_to_pifrac(rad, max_denominator=8000):
    pifrac = Fraction(rad / pi).limit_denominator(max_denominator)
    if pifrac == 0:
        return '0'
    num = {1: '', -1: '-'}.get(pifrac.numerator, str(pifrac.numerator))
    denom = '/{}'.format(pifrac.denominator) if pifrac.denominator != 1 else ''
    return 'pi'.join((num, denom))

def test_pi(max_denominator=100):
    testcases = ['0', '1', '0.39', '3', 'pi/8', 'pi/4', '-pi/4', '3*pi/4', '-3*pi/4', '-pi', 'pi', '2*pi', '6.28']
    for s in testcases:
        print('{:10s} ----> {}'.format(s, rad_to_pifrac(eval(s), max_denominator)))

def main(argv):
    (fst_file, snd_file) = get_args(argv)
    fst_matrix = read_matrix(fst_file)
    snd_matrix = read_matrix(snd_file)
    #diff_matrix = snd_matrix - fst_matrix
    fst_theta = euler_angles(fst_matrix)
    snd_theta = euler_angles(snd_matrix)
    #(theta_x, theta_y, theta_z) = euler_angles(diff_matrix[:3,:3])
    fst_transl = fst_matrix[:3, 3]
    snd_transl = snd_matrix[:3, 3]
    diff = array([*fst_transl, *fst_theta]) - array([*snd_transl, *snd_theta])
    print('translation (x,y,z), rotation Euler angles (ax,ay,az): ', *diff)

if __name__ == '__main__':
    main(sys.argv)
