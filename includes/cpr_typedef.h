#ifndef __DEF_CPR_TYPEDEF_H__
# define __DEF_CPR_TYPEDEF_H__

#include <map>
#include <vector>
#include <Eigen/Core>   // Eigen::Matrix<Double,x,x,RowMajor>
//#include <pcl/point_types.h>
//#include <pcl/point_cloud.h>
#include <pcl/segmentation/supervoxel_clustering.h>  // pcl::Supervoxel<P>::Ptr

// Types for points and point clouds
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;

// Types for list of supervoxels and adjacency of supervoxels
typedef std::uint32_t KeyT;
typedef std::map <KeyT, pcl::Supervoxel<PointT>::Ptr> SupervoxelClusters;
typedef std::multimap<KeyT, KeyT>   SupervoxelAdjacency;

// Types for ESF descriptors of supervoxel clusters
typedef std::vector<float>      ESFHist;
typedef std::map<KeyT, ESFHist> ESFDescriptors;

// Types for descriptors of edges between adjacent supervoxels
typedef std::tuple<double, double, double, double> EdgeFeature;
typedef std::map<
                std::pair<KeyT, KeyT>,
                EdgeFeature
                > EdgeDescriptors;

typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixDouble;
typedef Eigen::Matrix<int,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> MatrixInt;


#endif /* __DEF_CPR_TYPEDEF_H__ */
