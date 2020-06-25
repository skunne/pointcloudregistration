#!/usr/bin/env python3

theta_str = ['0.0', '0.3141592653589793', '0.6283185307179586', '0.9424777960769379', '1.2566370614359172', '1.5707963267948966']
theta = [0, 0.3141592653589793, 0.6283185307179586, 0.9424777960769379, 1.2566370614359172, 1.5707963267948966]
# dist_notransform = [0, 2.401976927977115, 6.498199946522295, 9.569503991021588, 8.776079405704339, 10.4816863215379]
# dist_inliers_notransform = [0, 2.15944342891211, 5.591539324192125, 11.618049527259295, 9.139919134946531, 9.853407848093253]

import sys

sys.path.insert(1, '../../scripts_python')

import distances_list_matched_pointclouds_csv
import matplotlib.pyplot as plt

clouds_dist_notransform = []
for i, angle in enumerate(theta_str):
    list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'matched_pointclouds/nuclei_r{}.pcd_src.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_dst.csv'.format(angle)])
    clouds_dist_notransform.append(list_of_d)
    plt.scatter([theta[i]-0.0025] * len(list_of_d), list_of_d, s=0.7, c='red')
dist_notransform = [sum(c)/len(c) for c in clouds_dist_notransform]

clouds_dist = []
for i, angle in enumerate(theta_str):
    list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'transforms/nuclei_r{}.pcd_computed.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_src.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_dst.csv'.format(angle)])
    clouds_dist.append(list_of_d)
    plt.scatter([theta[i]-0.0075] * len(list_of_d), list_of_d, s=0.7, c='orange')
dist = [sum(c)/len(c) for c in clouds_dist]

clouds_dist_icpransac = []
for i, angle in enumerate(theta_str):
    list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'transforms/nuclei_r{}.pcd_inliers_computed.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_inliers_src.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_inliers_dst.csv'.format(angle)])
    clouds_dist_icpransac.append(list_of_d)
    plt.scatter([theta[i]+0.0025] * len(list_of_d), list_of_d, s=0.7, c='purple')
dist_icpransac = [sum(c)/len(c) for c in clouds_dist_icpransac]

clouds_dist_icpransac_notransform = []
for i, angle in enumerate(theta_str):
    list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'matched_pointclouds/nuclei_r{}.pcd_inliers_src.csv'.format(angle), 'matched_pointclouds/nuclei_r{}.pcd_inliers_dst.csv'.format(angle)])
    clouds_dist_icpransac_notransform.append(list_of_d)
    plt.scatter([theta[i]+0.0075] * len(list_of_d), list_of_d, s=0.7, c='blue')
clouds_dist_icpransac_notransform = [sum(c)/len(c) for c in clouds_dist_icpransac_notransform]

import matplotlib.pyplot as plt

plt.plot(theta, dist, label='matched,registered', c='orange')
plt.plot(theta, dist_notransform, label='matched', c='red')
plt.plot(theta, dist_icpransac, label='matched,icpransac,registered', c='purple')
plt.plot(theta, clouds_dist_icpransac_notransform, label='matched,icpransac', c='blue')
plt.legend(loc='best')
plt.xlabel('rotation angle')
plt.ylabel('average distance')
plt.savefig('average_distances.png')
plt.show()
