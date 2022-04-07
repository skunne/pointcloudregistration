#!/usr/bin/env python3

import sys
from math import sqrt

def list_of_pcdfile(filename):
    pointcloud = []
    with open(filename, 'r') as f:
        for line in f:
            row = line.split()
            if row[0] not in ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']:
                p = [float(x) for x in row[:3]]
                pointcloud.append(p)
    return pointcloud

def squared_euclidean_distance(p,q):
    return sum((xp - xq) * (xp - xq) for xp,xq in zip(p,q))

def compute_diameter(pointcloud):
    squared_diameter = max(squared_euclidean_distance(p,q) for i,p in enumerate(pointcloud) for q in pointcloud[i+1:])
    return sqrt(squared_diameter)

def print_usage():
    print('SYNOPSIS')
    print()
    print('{} pcdfile'.format(sys.argv[0]))
    print('      Calculate the diameter of the pointcloud in <pcdfile>,')
    print('      i.e. the largest distance between two points in the cloud.')
    sys.exit(1)

def main():
    if len(sys.argv) == 2:
        print(compute_diameter(list_of_pcdfile(sys.argv[1])))
    else:
        print_usage()

if __name__=='__main__':
    main()
