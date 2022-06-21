#!/usr/bin/env python3

theta_str = ['0.0', '0.3141592653589793', '0.6283185307179586', '0.9424777960769379', '1.2566370614359172', '1.5707963267948966']
theta = [0, 0.3141592653589793, 0.6283185307179586, 0.9424777960769379, 1.2566370614359172, 1.5707963267948966]

import sys

sys.path.insert(1, '../../scripts_python')

import distances_list_matched_pointclouds_pcd
import matplotlib.pyplot as plt
import math  # sqrt to calculate rmse

# valve / nuclei
# allpoints / thinned
# rotation 0 / ??
# full / cropped

# problem: pour valve, seulement rotation

def calc_dist(mfile, srcfile, dstfile):
    if mfile is None:
        list_of_d = distances_list_matched_pointclouds_pcd.main(['./distances_list_matched_pointclouds_csv.py', srcfile, dstfile])
    else:
        list_of_d = distances_list_matched_pointclouds_pcd.main(['./distances_list_matched_pointclouds_csv.py', mfile, srcfile, dstfile])
    avg_d = sum(list_of_d) / len(list_of_d)
    rmse = math.sqrt(sum(x*x for x in list_of_d) / len(list_of_d))
    return avg_d, rmse, list_of_d

prefix = ['nuclei', 'valve']
cropped = ['', '_cropped']
thinned = ['', '_thinned']
rotated = ['', '_rotatedpi4', '_rotatedpi2']
infix = '.pcd'
srcdst = ['_src', '_dst']
suffix = '.csv'

def main(criterion):
    for p in prefix:
        x = []
        y_ouralgo = []
        y_ouralgo_inliers = []
        y_icpransac = []
        for r in rotated:
            for c in cropped:
                for t in thinned:
                    x.append(('C ' if c else '') + ('S ' if t else '') + ('pi/4' if 'pi4' in r else 'pi/2' if r else ''))
                    # matrix_computed = 'transforms/'+p+c+t+r+'.pcd_computed.csv'
                    # source = 'pointclouds/'+ p + c + t +'.pcd'
                    # dest   = 'pointclouds/'+ p + c + t + r +'.pcd'
                    # avg_d, rmse, cloud_d = calc_dist(matrix_computed, source, dest)
                    # y_ouralgo.append(avg_d)

                    matrix_computed = 'transforms/'+p+c+t+r+'.pcd_inliers_computed.csv'
                    source = 'pointclouds/'+p+c+t+'.pcd'
                    dest   = 'pointclouds/'+p+c+t+r+'.pcd'
                    our_avg_d, our_rmse, our_cloud_d = calc_dist(matrix_computed, source, dest)
                    #y_ouralgo_inliers.append(rmse)#rmse)#avg_d)
                    #plt.scatter([x[-1] for _ in our_cloud_d], our_cloud_d, c='green', s=0.7)

                    matrix_icpransac = 'transforms/'+p+c+t+r+'.pcd_icpransac.csv'
                    source = 'pointclouds/'+p+c+t+'.pcd'
                    dest   = 'pointclouds/'+p+c+t+r+'.pcd'
                    icp_avg_d, icp_rmse, icp_cloud_d = calc_dist(matrix_icpransac, source, dest)
                    #y_icpransac.append(rmse)#rmse)#avg_d)

                    if (criterion=='rmse'):
                        y_ouralgo_inliers.append(our_rmse)
                        y_icpransac.append(icp_rmse)
                    else:
                        y_ouralgo_inliers.append(our_avg_d)
                        y_icpransac.append(icp_avg_d)

        # plt.scatter(x, y_ouralgo, label='our algo without ransac on {}'.format(p))
        # plt.plot(x, y_ouralgo)
        #plt.scatter(x, y_ouralgo_inliers, label='our algo on {}'.format(p))
        plt.plot(x, y_ouralgo_inliers, label='our algo')
        #plt.scatter(x, y_icpransac, label='icp/ransac on {}'.format(p))
        plt.plot(x, y_icpransac, label='icp/ransac')

        plt.legend(loc='best')
        plt.xlabel(p)
        if (criterion == 'rmse'):
            plt.ylabel('root mean squared pointwise distance')
        else:
            plt.ylabel('average distance (um)')
        plt.gcf().set_size_inches(16, 12)
        plt.savefig('evaluation_performances_{}.png'.format(p), dpi=100)
        #plt.ylim(0,10)
        plt.show()

if __name__=='__main__':
    criterion = sys.argv[1] if len(sys.argv) > 1 else 'avg'
    main(criterion)
