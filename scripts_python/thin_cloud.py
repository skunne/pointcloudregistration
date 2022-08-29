#!/usr/bin/env python3

import sys          # argv, exit()

def print_usage_and_exit(cmd):
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(cmd))
    print('    print this help message and exit')
    print()
    print('{} <ifile> <ofile> [<n>]'.format(cmd))
    print('    1) read .pcd file <ifile>')
    print('    2) remove every n-1 out of n points')
    print('    3) write resulting point cloud to file ofile in pcd format')
    print('    default value: n=3')
    #print('    Parameters xmin, ymin and zmin default to 0')
    print()
    sys.exit()

def get_args(argv):
    if (len(argv) > 1 and argv[1] in ['-h', '--help']):
        print_usage_and_exit()
    elif len(argv) > 2:
        infilename = argv[1]
        outfilename = argv[2]
        if len(argv) > 3:
            n = int(argv[3])
            return (infilename, outfilename, n)
    return (infilename, outfilename, None)

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

# def dont_kill_it(i):
#     return (
#         i % 3 == 0
#     )

def filter_pointcloud(pointcloud, n=3):
    if n==None:
        n = 3
    #return [(x,y,z,rgba) for (i,(x,y,z,rgba)) in enumerate(pointcloud) if dont_kill_it(i)]
    return pointcloud[::n]

def fix_header(header, new_nb_points):
    new_header = []
    for line in header:
        row = line.split()
        if row[0] in ['WIDTH', 'POINTS']:
            new_header.append('{} {}\n'.format(row[0], new_nb_points))
        else:
            new_header.append(line)
    return new_header

def write_file(header, pointcloud, outfilename):
    with open(outfilename, 'w') as f:
        for line in header:
            f.write(line)
        for (x,y,z,rgba) in pointcloud:
            f.write("{} {} {} {}\n".format(x, y, z, rgba))

def main(argv):
    (infilename, outfilename, n) = get_args(argv)
    header, pointcloud = read_file(infilename)
    filtered_pointcloud = filter_pointcloud(pointcloud, n)
    header = fix_header(header, len(filtered_pointcloud))
    write_file(header, filtered_pointcloud, outfilename)

if __name__ == '__main__':
    main(sys.argv)
