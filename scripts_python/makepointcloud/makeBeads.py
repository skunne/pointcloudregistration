import random
import makeSnowman as snow
from math import sqrt


def drawBeads(centers, r, n):
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

def printToMeta(outf, min_distance):
    outf.write('filename pointclouds/beads.pcd\n')
    outf.write('voxel_resolution 0.1\n')   # approx = sqrt((4 pi r^2)/n) where r is radius of sphere and n is nb points per sphere
    outf.write('seed_resolution {}\n'.format(min_distance/2))      # seed_resolution > voxel_resolution
    outf.write('color_importance 0.2\n')    # could be 0, we did not use colour
    outf.write('spatial_importance 1.0\n')
    outf.write('normal_importance 1.0\n')
    outf.write('adjacency_filename output/beads.adj\n')

def minDistance_helper(c, cs):
    (xa,ya,za) = c
    return min((xa-xb)**2 + (ya-yb)**2 + (za-zb)**2 for (xb,yb,zb) in cs)

def minDistance(centers):
    c_cs_lst = [(centers[i], centers[i+1:]) for i in range(len(centers))]
    return sqrt(min(minDistance_helper(c, cs) for (c,cs) in c_cs_lst if cs))

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} [nb_beads [nb_points]]'.format(sys.argv[0]))
    print('    Generate a point cloud made of <nb_points> points randomly spread')
    print('    over the surface of <nb_beads> spheres with random centers')
    print('    Default parameters:')
    print('        nb_beads = 20')
    print('        nb_points = nb_beads * 10')
    print()
    sys.exit()

def get_args():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']) or len(sys.argv) > 3:
        print_usage_and_exit()
    nb_beads = 20
    if len(sys.argv) > 1:
        nb_nuclei = int(sys.argv[1])
    nb_points = 10 * nb_beads
    if len(sys.argv) > 2:
        nb_points = int(sys.argv[2])
    return nb_nuclei, nb_points

def main():
    random.seed()
    nb_beads, nb_points = get_args()
    nb_beads = 20
    nb_points = 200
    centers = chooseCenters(10.0, 10.0, 10.0, nb_beads)
    min_distance = minDistance(centers)
    print('Minimum distance between beads: {}\nBeads radius: {}'.format(min_distance, 0.05))
    pointcloud = drawBeads(centers, 0.05, nb_points)
    with open('pointclouds/beads.pcd', 'w') as outf:
        snow.printToPcd(outf, pointcloud)
    with open('metadata/beads.meta', 'w') as outf:
        printToMeta(outf, min_distance)

if __name__ == '__main__':
    main()
