#!/usr/bin/env python3

import sys                          # sys.argv
import xml.etree.ElementTree as ET  # ET.parse

default_xml_filename = 'pointclouds/roiCoeurMicroCT_pointcloud_5.xml'

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

# tree = ET.parse(filename)
# root = tree.getroot()
# for child in root:
#     for grandchild in child:
#         if grandchild.tag == 'position':
#             for coord in grandchild:
#                 print(coord.tag, coord.text, end=' ')
#             print()
# points = [[int(coord.text) for coord in point] for roi in root for point in roi if point.tag == 'position']
# print(points[:5], points[-5:])

def read_xml(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    points = [[int(coord.text) for coord in point] for roi in root for point in roi if point.tag == 'position']
    return points

def write_pcd(points, pcd_filename):
    nb_points = len(points)
    with open(pcd_filename, 'w') as f_out:
        f_out.write(default_header.format(nb_points, nb_points))
        for i,(x,y,z) in enumerate(points):
            f_out.write('{} {} {} {}\n'.format(x,y,z,default_colour))

def main(argv):
    if len(argv) in [2,3]:
        if len(argv) == 3:
            xml_in = argv[1]
            pcd_out = argv[2]
        elif len(argv) == 2:
            xml_in = default_xml_filename
            pcd_out = argv[1]
        print('XML IN: {}\nPCD OUT: {}'.format(xml_in, pcd_out))
        points = read_xml(xml_in)
        write_pcd(points, pcd_out)
    else:
        print('USAGE:')
        print('  {} xmlfile pcdfile'.format(argv[0]))

if __name__=='__main__':
    main(sys.argv)
