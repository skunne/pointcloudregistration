#ifndef __TEST_ICP_RANSAC_H__
# define __TEST_ICP_RANSAC_H__

#include <pcl/point_types.h>
#include <Eigen/Core>

double icpransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dest, Eigen::Matrix4f &transform);


#endif /* __TEST_ICP_RANSAC_H__ */
