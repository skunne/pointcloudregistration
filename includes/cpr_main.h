#ifndef __DEF_MAIN_H__
# define __DEF_MAIN_H__

#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/supervoxel_clustering.h>

// compute ESF descriptors
#include <pcl/features/esf.h>
#define HISTOGRAM_SIZE  (640)

//VTK include needed for drawing graph lines
#include <vtkPolyLine.h>

//VTK include needed to input vtk cloud file
#include <pcl/io/vtk_lib_io.h>
#include <vtkGenericDataObjectReader.h>
//#include <vtkStructuredGrid.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
//#include <string>

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



#endif /* __DEF_MAIN_H__ */
