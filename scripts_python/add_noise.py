#!/usr/bin/env python3

import sys          # argv, exit()
import random       # gauss

def print_usage_and_exit(cmd):
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(cmd))
    print('    print this help message and exit')
    print()
    print('{} ifile ofile [stdev]'.format(cmd))
    print('    1) read .pcd file ifile')
    print('    2) add Gaussian noise N(0, stdev**2) to every point')
    print('    3) write resulting point cloud to file <ofile> in pcd format')
    print('    Parameter stdev default to 1 and should be given in same distance units as point coordinates')
    print()
    sys.exit()

def add_noise(pointcloud, stdev):
    return [(x+random.gauss(0,stdev), y+random.gauss(0,stdev), z+random.gauss(0,stdev), rgba)
            for (x,y,z,rgba) in pointcloud]

def get_args(argv):
    if (len(argv) > 1 and argv[1] in ['-h', '--help']):
        print_usage_and_exit(argv[0])
    elif not (3 <= len(argv) <= 4):
        print_usage_and_exit(argv[0])
    elif (len(argv) == 3):
        infilename = argv[1]
        outfilename = argv[2]
        stdev = 1
    elif (len(argv) == 4):
        infilename = argv[1]
        outfilename = argv[2]
        try:
            stdev = float(argv[3])
        except ValueError:
            print_usage_and_exit(argv[0])
    return (infilename, outfilename, stdev)

def read_file(infilename):
    header = []
    pointcloud = []
    with open(infilename, 'r') as f:
        for line in f:
            row = line.split()
            if row[0] in ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']:
                header.append(line)
            else:
                (sx,sy,sz,srgba) = row
                (x,y,z,rgba) = (float(sx), float(sy), float(sz), int(srgba))
                pointcloud.append((x,y,z,rgba))
    return header, pointcloud

def write_file(header, pointcloud, outfilename):
    with open(outfilename, 'w') as f:
        for line in header:
            f.write(line)
        for (x,y,z,rgba) in pointcloud:
            f.write("{} {} {} {}\n".format(x, y, z, rgba))

def main(argv):
    (infilename, outfilename, stdev) = get_args(argv)
    header, pointcloud = read_file(infilename)
    noisy_pointcloud = add_noise(pointcloud, stdev)
    write_file(header, filtered_pointcloud, outfilename)

if __name__ == '__main__':
    main(sys.argv)
