import random
import makeSnowman as snow


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

def main():
    random.seed()
    centers = chooseCenters(10.0, 10.0, 10.0, 5)
    pointcloud = drawNuclei(centers, 1.0, 2400)
    with open('pointclouds/nuclei.pcd', 'w') as outf:
        snow.printToPcd(outf, pointcloud)
    with open('metadata/nuclei.meta', 'w') as outf:
        printToMeta(outf)

if __name__ == '__main__':
    main()
