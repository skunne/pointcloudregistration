#!/usr/bin/env python3

# theta_str = ['0.0', '0.3141592653589793', '0.6283185307179586', '0.9424777960769379', '1.2566370614359172', '1.5707963267948966']
# theta = [0, 0.3141592653589793, 0.6283185307179586, 0.9424777960769379, 1.2566370614359172, 1.5707963267948966]
# dist_notransform = [0, 2.401976927977115, 6.498199946522295, 9.569503991021588, 8.776079405704339, 10.4816863215379]
# dist_inliers_notransform = [0, 2.15944342891211, 5.591539324192125, 11.618049527259295, 9.139919134946531, 9.853407848093253]

import sys          # path.insert
import itertools    # product

sys.path.insert(1, '../../scripts_python')

import distance_matched_pointclouds_pcd
import matplotlib.pyplot as plt

# clouds_dist_notransform = []
# for i, angle in enumerate(theta_str):
#     list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'matched_pointclouds/valve_r{}.pcd_src.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_dst.csv'.format(angle)])
#     clouds_dist_notransform.append(list_of_d)
#     #plt.scatter([theta[i]-0.0025] * len(list_of_d), list_of_d, s=0.7, c='red')
# dist_notransform = [sum(c)/len(c) for c in clouds_dist_notransform]

# def calc_dists(mfile, srcfile, dstfile, offset, colour):
#     clouds_dist = []
#     for i, angle in enumerate(theta_str):
#         if mfile is None:
#             list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', srcfile.format(angle), dstfile.format(angle)])
#         else:
#             list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', mfile.format(angle), srcfile.format(angle), dstfile.format(angle)])
#         clouds_dist.append(list_of_d)
#         plt.scatter([theta[i]-offset] * len(list_of_d), list_of_d, s=0.2, c=colour)
#     dist = [sum(c)/len(c) for c in clouds_dist]
#     return (dist, clouds_dist)

# dist,clouds_dist = calc_dists('transforms/valve_r{}.pcd_computed.csv', 'matched_pointclouds/valve_r{}.pcd_src.csv', 'matched_pointclouds/valve_r{}.pcd_dst.csv', -0.0075, 'orange')
# plt.plot(theta, dist, label='graph', c='orange')
#
# dist_icpransac,clouds_dist_icpransac = calc_dists('transforms/valve_r{}.pcd_inliers_computed.csv', 'matched_pointclouds/valve_r{}.pcd_inliers_src.csv', 'matched_pointclouds/valve_r{}.pcd_inliers_dst.csv', 0.0025, 'purple')
# plt.plot(theta, dist_icpransac, label='graph,icpransac', c='purple')

# dist_original, clouds_dist_original = calc_dists('transforms/valve_r{}.pcd_computed.csv', 'pointclouds/valve.csv', 'pointclouds/valve_r{}.csv', -0.005, 'red')
# plt.plot(theta, dist_original, label='original', c='red')
#
# print('Nombre de points: {}'.format(len(clouds_dist_original[0])))
#
# dist_original_icpransac, clouds_dist_orig_icpr = calc_dists('transforms/valve_r{}.pcd_inliers_computed.csv', 'pointclouds/valve.csv', 'pointclouds/valve_r{}.csv', 0.005, 'blue')
# plt.plot(theta, dist_original_icpransac, label='original,icpransac', c='blue')

# dist_original_notransfo, clouds_dist_orig_notransfo = calc_dists(None, 'pointclouds/valve.csv', 'pointclouds/valve_r{}.csv', 0.005, 'green')
# plt.plot(theta, dist_original_notransfo, label='no transfo', c='green')

# clouds_dist = []
# for i, angle in enumerate(theta_str):
#     list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'transforms/valve_r{}.pcd_computed.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_src.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_dst.csv'.format(angle)])
#     clouds_dist.append(list_of_d)
#     plt.scatter([theta[i]-0.0075] * len(list_of_d), list_of_d, s=0.7, c='orange')
# dist = [sum(c)/len(c) for c in clouds_dist]
#
# clouds_dist_icpransac = []
# for i, angle in enumerate(theta_str):
#     list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'transforms/valve_r{}.pcd_inliers_computed.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_inliers_src.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_inliers_dst.csv'.format(angle)])
#     clouds_dist_icpransac.append(list_of_d)
#     plt.scatter([theta[i]+0.0025] * len(list_of_d), list_of_d, s=0.7, c='purple')
# dist_icpransac = [sum(c)/len(c) for c in clouds_dist_icpransac]

# clouds_dist_icpransac_notransform = []
# for i, angle in enumerate(theta_str):
#     list_of_d = distances_list_matched_pointclouds_csv.main(['./distances_list_matched_pointclouds_csv.py', 'matched_pointclouds/valve_r{}.pcd_inliers_src.csv'.format(angle), 'matched_pointclouds/valve_r{}.pcd_inliers_dst.csv'.format(angle)])
#     clouds_dist_icpransac_notransform.append(list_of_d)
#     #plt.scatter([theta[i]+0.0075] * len(list_of_d), list_of_d, s=0.7, c='blue')
# clouds_dist_icpransac_notransform = [sum(c)/len(c) for c in clouds_dist_icpransac_notransform]



import matplotlib.pyplot as plt

for imgname in ['nuclei', 'valve', 'heart']:
    x_axis = [''.join(triplet) for triplet in itertools.product(['', 'pi/4', 'pi/2'], ['', 'C'], ['', 'S'])]
    transfo_combs = list(itertools.product(['', '_rotatedpi4', '_rotatedpi2'], ['', '_cropped'], ['', '_thinned']))
    srcfiles = [''.join(['pointclouds/', imgname, '.pcd']) for r,c,s in transfo_combs]
    dstfiles = [''.join(['pointclouds/', imgname ,r, '.pcd']) for r,c,s in transfo_combs]
    matfiles_ours = [''.join(['transforms/', imgname, c,s,r, '.pcd_computed.csv']) for r,c,s in transfo_combs]
    matfiles_icp = [''.join(['transforms/', imgname, c,s,r, '.pcd_icpransac.csv']) for r,c,s in transfo_combs]
    mat_ours = [distance_matched_pointclouds_pcd.get_matrix(mat) for mat in matfiles_ours]
    mat_icp = [distance_matched_pointclouds_pcd.get_matrix(mat) for mat in matfiles_icp]
    distances_ours = [distance_matched_pointclouds_pcd.compute_average_distance(src, dst, mat) for mat, src, dst in zip(mat_ours, srcfiles, dstfiles)]
    distances_icp = [distance_matched_pointclouds_pcd.compute_average_distance(src, dst, mat) for mat, src, dst in zip(mat_icp, srcfiles, dstfiles)]

    plt.plot(x_axis, distances_ours, label='{}, our algo'.format(imgname))
    plt.plot(x_axis, distances_icp, label='{}, icp ransac'.format(imgname))
#plt.plot(theta, dist_notransform, label='graph', c='red')

#plt.plot(theta, clouds_dist_icpransac_notransform, label='graph,icpransac', c='blue')
plt.legend(loc='best')
plt.xlabel('point cloud')
plt.ylabel('average distance')
plt.savefig('average_distances.png')
#plt.ylim(0,20)
#plt.savefig('average_distances_rapport1an.png')
plt.show()
