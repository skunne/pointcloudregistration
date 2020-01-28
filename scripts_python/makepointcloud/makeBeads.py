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
    outf.write('voxel_resolution 0.01\n')   # approx = sqrt((4 pi r^2)/n) where r is radius of sphere and n is nb points per sphere
    outf.write('seed_resolution {}\n'.format(min_distance))      # seed_resolution > voxel_resolution
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

def main():
    random.seed()
    centers = chooseCenters(10.0, 10.0, 10.0, 20)
    min_distance = minDistance(centers)
    print('Minimum distance between beads: {}\nBeads radius: {}'.format(min_distance, 0.05))
    pointcloud = drawBeads(centers, 0.05, 200)
    with open('pointclouds/beads.pcd', 'w') as outf:
        snow.printToPcd(outf, pointcloud)
    with open('metadata/beads.meta', 'w') as outf:
        printToMeta(outf, min_distance)

if __name__ == '__main__':
    main()
