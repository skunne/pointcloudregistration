import random               # random.seed()
import makeSnowman as snow  # addPointsOnSphere()
import sys                  # argv, exit()

def drawNuclei(centers, r, n):
    pointcloud = []
    for i in range(n):
        (cx,cy,cz) = random.choice(centers)
        snow.addPointsOnSphere(cx,cy,cz, r, 1, pointcloud)
    return pointcloud

def chooseCenters(xmax, ymax, zmax, n):
    centers = []
    for i in range(n):
        (x,y,z) = random.uniform(0,xmax), random.uniform(0,ymax), random.uniform(0,zmax)
        centers.append((x,y,z))
    return centers

def printToMeta(outf):
    outf.write('filename pointclouds/nuclei.pcd\n')
    outf.write('voxel_resolution 0.2\n')   # approx = sqrt((4 pi r^2)/n) where r is radius of sphere and n is nb points per sphere
    outf.write('seed_resolution 1.0\n')      # seed_resolution > voxel_resolution
    outf.write('color_importance 0.2\n')    # could be 0, we did not use colour
    outf.write('spatial_importance 1.0\n')
    outf.write('normal_importance 1.0\n')
    outf.write('adjacency_filename output/nuclei.adj\n')

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    Print this help message and exit')
    print()
    print('{} [nb_nuclei [nb_points]]'.format(sys.argv[0]))
    print('    Generate a point cloud made of <nb_points> points randomly spread')
    print('    over the surface of <nb_nuclei> spheres with random centers')
    print('    Default parameters:')
    print('        nb_nuclei = 5')
    print('        nb_points = nb_nuclei * 480')
    print()
    sys.exit()

def get_args():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']) or len(sys.argv) > 3:
        print_usage_and_exit()
    nb_nuclei = 5
    if len(sys.argv) > 1:
        nb_nuclei = int(sys.argv[1])
    nb_points = 480 * nb_nuclei
    if len(sys.argv) > 2:
        nb_points = int(sys.argv[2])
    return nb_nuclei, nb_points

pcdfilename = 'nuclei.pcd'
metafilename = 'nuclei.meta'

def main():
    random.seed()
    nb_nuclei, nb_points = get_args()
    centers = chooseCenters(10.0, 10.0, 10.0, nb_nuclei)
    pointcloud = drawNuclei(centers, 1.0, nb_points)
    with open(pcdfilename, 'w') as outf:
        snow.printToPcd(outf, pointcloud)
    with open(metafilename, 'w') as outf:
        printToMeta(outf)

if __name__ == '__main__':
    main()
