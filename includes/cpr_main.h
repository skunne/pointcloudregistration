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
typedef std::map<
                std::pair<KeyT, KeyT>,
                std::tuple<double, double, double, double>
                > EdgeDescriptors;

/*
** loadfiles.cpp
*/



/*
** visualisation.cpp
*/
void visualisation(pcl::SupervoxelClustering<PointT> const &super, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency const &supervoxel_adjacency);
void addSupervoxelConnectionsToViewer (PointT const &supervoxel_center,
                                       PointCloudT const &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr &viewer);




#endif /* __DEF_MAIN_H__ */
