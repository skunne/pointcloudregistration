#!/usr/bin/env python3

import matplotlib.pyplot as plt
from math import pi,sin,cos
# import random
import sys

#default values:
xmin = -10
xmax = 10
ymin = -10
ymax = 10
default_rotation_angle = 1.1 * pi / 2.0

nb_points = 6

# def apply_linear_horizontal(M, pointcloud):
#     ((a, c), (b, d)) = M
#     return [(a*x+c*y, b*x+d*y, z, rgba) for (x,y,z,rgba) in pointcloud]

# def apply_translat(a,b, X,Y):
#     return [x+a for x in X], [y+b for y in Y]

def apply_affine(M, pointcloud):
    ((a, d, g, tx), (b, e, h, ty), (c, f, i, tz), p) = M
    assert (p == (0,0,0,1))
    return [(a*x+d*y+g*z+tx, b*x+e*y+h*z+ty, c*x+f*y+i*z+tz, rgba) for (x,y,z,rgba) in pointcloud]

def make_rotation_matrix(theta):
    costheta = cos(theta)
    sintheta = sin(theta)
    return (
        (costheta,-sintheta,0,0),
        (sintheta, costheta,0,0),
        (0,0,1,0),
        (0,0,0,1)
    )

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

# def fix_header(header, new_nb_points):
#     new_header = []
#     for line in header:
#         row = line.split()
#         if row[0] in ['WIDTH', 'POINTS']:
#             new_header.append('{} {}\n'.format(row[0], new_nb_points))
#         else:
#             new_header.append(line)
#     return new_header

def write_file(header, pointcloud, outfilename):
    print("Writing {} header lines and {} points to {}".format(len(header), len(pointcloud), outfilename))
    with open(outfilename, 'w') as f:
        for line in header:
            #print(line, end='')
            f.write(line)
        for (x,y,z,rgba) in pointcloud:
            #print("{} {} {} {}".format(x, y, z, rgba))
            f.write("{} {} {} {}\n".format(x, y, z, rgba))

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} ifile ofile [theta [mfile]]'.format(sys.argv[0]))
    print('    Read a point cloud from pcd file <ifile>')
    print('    Apply a rotation of angle theta in the plane xy to every point')
    print('    Write result in pcd format to <ofile>')
    print('    If specified, write transfo matrix to file <mfile>')
    print('    Default values:')
    print('        theta = 1.73')
    print()
    sys.exit()

def write_matrix(matrix, filename):
    # print('matrix:')
    # print(matrix)
    with open(filename, 'w') as f:
        for row in matrix:
            for x in row[:-1]:
                f.write("{},".format(x))
            f.write("{}\n".format(row[-1]))

def get_args(argv):
    infile = 'in.pcd'
    outfile = 'out.pcd'
    theta = default_rotation_angle
    matrixfile = None
    nb_args = len(argv) - 1
    if (nb_args == 0 or sys.argv[1] in ['-h', '--help']):
        print_usage_and_exit()
    if (nb_args > 1):
        (infile, outfile) = (sys.argv[1], sys.argv[2])
        if (nb_args > 2):
            theta = float(sys.argv[3])
            if (nb_args > 3):
                matrixfile = sys.argv[4]
    return (infile, outfile, theta, matrixfile)

def main(argv):
    (infilename, outfilename, rotation_angle, matrixoutfilename) = get_args(argv)
    header, pointcloud = read_file(infilename)
    rotation_matrix = make_rotation_matrix(rotation_angle)
    #rotation_matrix = ((1,0,0,10),(0,1,0,10),(0,0,1,10),(0,0,0,1))
    rotated_pointcloud = apply_affine(rotation_matrix, pointcloud)
    #header = fix_header(header, len(filtered_pointcloud))
    write_file(header, rotated_pointcloud, outfilename)
    if matrixoutfilename:
        write_matrix(rotation_matrix, matrixoutfilename)

if __name__=='__main__':
    main(sys.argv)



# def make_custom_data():
#     return (
#         [ 5, 0,10, 5, 5, 1],
#         [ 0, 5, 5,10,18,24]
#     )
#
# def make_random_data(n):
#     return (
#         [random.uniform(0.9*xmin,0.9*xmax) for i in range(n)],
#         [random.uniform(0.9*ymin,0.9*ymax) for i in range(n)]
#     )
#
# def plotplot(X,Y, filename):
#     #fig,ax=plt.subplots()
#     plt.figure(figsize=(8,8))
#     plt.scatter(X,Y,c='blue', s=50)
#     plt.axis('off')
#     #plt.set_aspect(1)
#     plt.xlim(xmin,xmax)
#     plt.ylim(ymin,ymax)
#     for i,(x,y) in enumerate(zip(X,Y)):
#         plt.annotate(str(i), xy=(x,y), xytext=(x+0.32,y+0.32), fontsize='xx-large')
#     plt.savefig(filename)
#     plt.show()
#     #plt.close(fig)
#
# def main():
#     #X,Y = make_custom_data()
#     random.seed()
#     X,Y = make_random_data(nb_points)
#     m = make_rotation_matrix(rotation_angle)
#     X2,Y2 = apply_linear(m, X,Y)
#     #X2,Y2 = apply_translat(35,0, X2,Y2)
#     plotplot(X,Y, 'img1.png')
#     plotplot(X2,Y2, 'img2.png')
