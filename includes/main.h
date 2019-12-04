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
int loadFile(char const *filename, bool is_pcd, PointCloudT::Ptr cloud);
int loadVTKFile(char const *filename, PointCloudT::Ptr cloud);
int loadPCDFile(char const *filename, PointCloudT::Ptr cloud);

/*
** clustering.cpp
*/
int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);
void performClustering(PointCloudT::Ptr cloud, pcl::SupervoxelClustering<PointT> &super, struct Params const *params, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);

/*
** connectedcomponents.cpp
*/
int getConnectedComponents(SupervoxelClusters const &vertices,
                          SupervoxelAdjacency const &edges,
                          std::vector<std::vector<KeyT>> &cc_list);
                          //std::map<KeyT, std::size_t> &cc_membership);

// add one vertex at center of each cc, and edges from these centers to connect the graph
void makeGraphConnected(SupervoxelClusters &vertices,
                        SupervoxelAdjacency &edges,
                        std::vector<std::vector<KeyT>> &cc_list);
                        //std::map<KeyT, std::size_t> &cc_membership);

/*
** features.cpp
*/
void calculateAnglesAndLength(PointT const &p1, PointT const &p2, double &angle_x, double &angle_y, double &angle_z, double &length);
void calculateESFDescriptors(SupervoxelClusters const &sv_clusters, ESFDescriptors &esf_descriptors);
void calculateEdgesDescriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, EdgeDescriptors &edge_descriptors);
void calculateDescriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, ESFDescriptors &esf_descriptors, EdgeDescriptors &edge_descriptors);

// save ESF descriptors to csv but (for now) ignores edge descriptors
void descriptorsToCSV(char const *name, ESFDescriptors esf_descriptors, EdgeDescriptors edge_descriptors);

/*
** visualisation.cpp
*/
void visualisation(pcl::SupervoxelClustering<PointT> const &super, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency const &supervoxel_adjacency);
void addSupervoxelConnectionsToViewer (PointT const &supervoxel_center,
                                       PointCloudT const &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr & viewer);




#endif /* __DEF_MAIN_H__ */
