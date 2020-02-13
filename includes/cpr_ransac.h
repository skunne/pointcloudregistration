#ifndef __DEF_CPR_RANSAC_H__
# define __DEF_CPR_RANSAC_H__

#include "cpr_main.h"


/*
** cpr_ransac.cpp
*/
void performRansacKeepInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_data,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_model,
      double threshold, //pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_points)
      std::vector<int> &inlier_indices);



#endif /* __DEF_CPR_RANSAC_H__ */
