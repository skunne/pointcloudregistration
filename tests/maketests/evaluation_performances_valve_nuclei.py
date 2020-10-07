#!/usr/bin/env python3

theta_str = ['0.0', '0.3141592653589793', '0.6283185307179586', '0.9424777960769379', '1.2566370614359172', '1.5707963267948966']
theta = [0, 0.3141592653589793, 0.6283185307179586, 0.9424777960769379, 1.2566370614359172, 1.5707963267948966]

import sys

sys.path.insert(1, '../../scripts_python')

import distances_list_matched_pointclouds_csv
import matplotlib.pyplot as plt
import math  # sqrt to calculate rmse

# valve / nuclei
# allpoints / thinned
# rotation 0 / ??
# full / cropped

# problem: pour valve, seulement rotation

def calc_dist(mfile, srcfile, dstfile):
    if mfile is None:
        list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', srcfile, dstfile])
    else:
        list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', mfile, srcfile, dstfile])
    avg_d = sum(list_of_d) / len(list_of_d)
    rmse = math.sqrt(sum(x*x for x in list_of_d) / len(list_of_d))
    return avg_d, list_of_d

prefix = ['nuclei', 'valve']
cropped = ['', '_cropped']
thinned = ['', '_thinned']
rotated = ['', '_rotated']
infix = '.pcd'
srcdst = ['_src', '_dst']
suffix = '.csv'

for p in prefix:
    x = []
    y_ouralgo = []
    y_icpransac = []
    for c in cropped:
        for t in thinned:
            for r in rotated:
                matrix_computed = 'transforms/'+p+c+t+r+infix+'_computed.csv'
                source = 'matched_pointclouds/'+p+c+t+r+infix+'_src.csv'
                dest = 'matched_pointclouds/'+p+c+t+r+infix+'_dst.csv'
                avg_d, cloud_d = calc_dist(matrix_computed, source, dest)
                x.append(c+t+r)
                y_ouralgo.append(avg_d)

    plt.scatter(x, y_ouralgo, label='our algo on {}'.format(p))
    plt.scatter([], [], label='icp/ransac on {}'.format(p))

    plt.legend(loc='best')
    plt.xlabel(p)
    plt.ylabel('average distance')
    plt.gcf().set_size_inches(16, 12)
    plt.savefig('evaluation_performances_{}.png'.format(p), dpi=100)
    #plt.ylim(0,10)
    plt.show()
