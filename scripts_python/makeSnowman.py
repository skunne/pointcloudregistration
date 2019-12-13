import math     # sin, cos
import random

twoPi = 2 * math.pi

def addPointsOnSphere(cx,cy,cz,r, n, pointcloud):
    for i in range(n):
        u = random.uniform(-1, 1)
        phi = math.acos(u)
        theta = random.uniform(0, twoPi)
        x = cx + r * math.sin(phi) * math.cos(theta)
        y = cy + r * math.sin(phi) * math.sin(theta)
        z = cz + r * math.cos(phi)
        pointcloud.append((x,y,z))
    return pointcloud

def addPointsOnHorizontalCone(cx,cy,cz, r, px, n, pointcloud):
    for i in range(n):
        h = random.triangular(0,1,0)
        theta = random.uniform(0,twoPi)
        x = cx + h * (px - cx)
        y = cy + r * (1-h) * math.sin(theta)
        z = cz + r * (1-h) * math.cos(theta)
        pointcloud.append((x,y,z))
    return pointcloud

def addPointsOnSegment(ax,ay,az,bx,by,bz, n, pointcloud):
    for i in range(n):
        u = random.uniform(0,1)
        x = ax + (bx - ax) * u
        y = ay + (by - ay) * u
        z = az + (bz - az) * u
        pointcloud.append((x,y,z))
    return pointcloud

def drawSnowman():
    pointcloud = []
    addPointsOnSphere(0,0,40, 40, 1600, pointcloud)        # abdomen
    addPointsOnSphere(0,0,40+40+20, 20, 400, pointcloud)  # torso
    addPointsOnSphere(0,0,80+40+10, 10, 100, pointcloud)  # head
    (thetaEye, phiEye) = (0.26, math.pi / 4)
    (xEye,yEye,zEye) = (10*math.sin(phiEye)*math.cos(-thetaEye),10*math.sin(phiEye)*math.sin(-thetaEye),120+10*math.cos(phiEye))
    addPointsOnSphere(xEye, -yEye, zEye, 4, 20, pointcloud)         # left eye
    addPointsOnSphere(xEye, yEye, zEye, 4, 20, pointcloud)          # right eye
    addPointsOnHorizontalCone(-8,0,120, 4, -16, 50, pointcloud)    # nose
    addPointsOnSegment(0,0,100, 0,50,100, 50, pointcloud)          # left arm
    addPointsOnSegment(0,0,100, 0,-50,100, 50, pointcloud)         # right arm
    return pointcloud

nbPointsTotal = 1600 + 400 + 100 + 20 + 20 + 50 + 50 + 50

def printToPcd(outf, pointcloud):
    outf.write("VERSION 0.7\n")
    outf.write("FIELDS x y z rgba\n")
    outf.write("SIZE 4 4 4 4\n")
    outf.write("TYPE F F F U\n")
    outf.write("COUNT 1 1 1 1\n")
    outf.write("WIDTH {:d}\n".format(nbPointsTotal))
    outf.write("HEIGHT 1\n")
    outf.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    outf.write("POINTS {:d}\n".format(nbPointsTotal))
    outf.write("DATA ascii\n")
    for p in pointcloud:
        outf.write('{:.2f} {:.2f} {:.2f} 4294967295\n'.format(p[0], p[1], p[2]))

def printToMeta(outf):
    outf.write('filename pointclouds/snowman.pcd\n')
    outf.write('voxel_resolution 3.0\n')   # approx = sqrt(n / (4 pi r^2)) where r,n are params used in call to addPointsOnSphere(.,.,.,r,n,.)
    outf.write('seed_resolution 15\n')      # seed_resolution > voxel_resolution
    outf.write('color_importance 0.2\n')    # could be 0, we did not use colour
    outf.write('spatial_importance 1.0\n')
    outf.write('normal_importance 1.0\n')
    outf.write('adjacency_filename output/snowman.adj\n')


if __name__ == '__main__':
    random.seed()
    pointcloud = drawSnowman()
    with open('pointclouds/snowman.pcd', 'w') as outf:
        printToPcd(outf, pointcloud)
    with open('metadata/snowman.meta', 'w') as outf:
        printToMeta(outf)
