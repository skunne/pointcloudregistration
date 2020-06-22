import matplotlib.pyplot as plt
from math import pi,sin,cos
# import random
import sys

#default values:
xmin = -10
xmax = 10
ymin = -10
ymax = 10
rotation_angle = 1.1 * pi / 2.0

nb_points = 6

def apply_linear(M, pointcloud):
    ((a, c), (b, d)) = M
    return [(a*x+c*y, b*x+d*y, z, rgba) for (x,y,z,rgba) in pointcloud]

def apply_translat(a,b, X,Y):
    return [x+a for x in X], [y+b for y in Y]

def make_rotation_matrix(theta):
    costheta = cos(theta)
    sintheta = sin(theta)
    return ((costheta,-sintheta), (sintheta, costheta))

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
    with open(outfilename, 'w') as f:
        for line in header:
            f.write(line)
        for (x,y,z,rgba) in pointcloud:
            f.write("{} {} {} {}\n".format(x, y, z, rgba))

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} ifile ofile [theta]'.format(sys.argv[0]))
    print('    Read a point cloud from pcd file <ifile>')
    print('    Apply a rotation of angle theta in the plane xy to every point')
    print('    Write result in pcd format to <ofile>')
    print('    Default values:')
    print('        theta = 1.73')
    print()
    sys.exit()

def write_matrix(matrix, filename):
    with open(filename, 'w') as f:
        for row in matrix:
            for x in row[:-1]:
                f.write("{},".format(x))
            print("{}\n".format(row[-1])

def get_args():
    infile = 'in.pcd'
    outfile = 'out.pcd'
    theta = rotation_angle
    matrixfile = None
    if (len(sys.argv) <= 1 or len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']):
        print_usage_and_exit()
    if (len(sys.argv) > 2):
        (infile, outfile) = (sys.argv[1], sys.argv[2])
    if (len(sys.argv) > 3):
        theta = float(sys.argv[3])
    if (len(sys.argv) > 4):
        matrixfile = sys.argv[4]
    return (infile, outfile, theta, matrixfile)

def main():
    (infilename, outfilename, rotation_angle, matrixoutfilename) = get_args()
    header, pointcloud = read_file(infilename)
    rotation_matrix = make_rotation_matrix(rotation_angle)
    rotated_pointcloud = apply_linear(rotation_matrix, pointcloud)
    #header = fix_header(header, len(filtered_pointcloud))
    write_file(header, rotated_pointcloud, outfilename)
    if matrixoutfilename:
        write_matrix(rotation_matrix, outfilename)

if __name__=='__main__':
    main()



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
