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

// Types
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointNCloudT;
typedef pcl::PointXYZL PointLT;
typedef pcl::PointCloud<PointLT> PointLCloudT;
typedef std::uint32_t KeyT;

// Types for list of supervoxels and adjacency of supervoxels
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

int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);
void perform_clustering(PointCloudT::Ptr cloud, pcl::SupervoxelClustering<PointT> &super, struct Params const *params, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);

// Calculate edge descriptors according to Huang et al 2017, Fig. 5
void calculate_angles_and_length(PointT const p1, PointT const p2, double &angle_x, double &angle_y, double &angle_z, double &length);
void calculate_esf_descriptors(SupervoxelClusters const &sv_clusters, ESFDescriptors &esf_descriptors);
void calculate_edges_descriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, EdgeDescriptors &edge_descriptors);
void calculate_descriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, ESFDescriptors &esf_descriptors, EdgeDescriptors &edge_descriptors);



void visualisation(pcl::SupervoxelClustering<PointT> const &super, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency const &supervoxel_adjacency);
void addSupervoxelConnectionsToViewer (PointT const &supervoxel_center,
                                       PointCloudT const &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);




#endif /* __DEF_MAIN_H__ */
