#!/usr/bin/env python3

import sys      # sys.argv
import open3d as o3d
import numpy as np

default_header='\n'.join([
    'VERSION 0.7',
    'FIELDS x y z rgba',
    'SIZE 4 4 4 4',
    'TYPE F F F U',
    'COUNT 1 1 1 1',
    'WIDTH {}',
    'HEIGHT 1',
    'VIEWPOINT 0 0 0 1 0 0 0',
    'POINTS {}',
    'DATA ascii\n'])

default_colour = 4294967295

def count_nb_points(csv_in):
    reader = csv.reader(csv_in)
    assert(next(reader)==['id','dimension_1','dimension_2','dimension_3'])   # skip 1 row header of csv file
    i = 0
    for row in reader:
        i += 1
    csv_in.seek(0)
    return(i)

def read_ply(ply_filename):
    pc = o3d.io.read_point_cloud(ply_filename)
    point_cloud_in_numpy = np.asarray(pc.points)
    return point_cloud_in_numpy

def write_pcd(csv_in, pcd_out):
    nb_points = count_nb_points(csv_in)
    pcd_out.write(default_header.format(nb_points, nb_points))
    reader = csv.reader(csv_in)
    assert(next(reader)==['id','dimension_1','dimension_2','dimension_3'])   # skip 1 row header of csv file
    i = 0;
    for row in reader:
        x,y,z = float(row[1]), float(row[2]), float(row[3])
        pcd_out.write('{} {} {} {}\n'.format(x,y,z,default_colour))
        i += 1
    return(i)

def main(argv):
    if len(argv) == 3:
        with open(argv[1], 'r') as csv_in:
            with open(argv[2], 'w') as pcd_out:
                write_pcd(csv_in, pcd_out)
    else:
        print('USAGE:')
        print('  {} csvfile pcdfile'.format(argv[0]))

if __name__=='__main__':
    main(sys.argv)
