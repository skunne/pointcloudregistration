
#ifndef __DEF_CPR_PROCESSEDPOINTCLOUD_H__
# define __DEF_CPR_PROCESSEDPOINTCLOUD_H__

#include <pcl/visualization/pcl_visualizer.h> // PCLVisualizer::Ptr viewer
#include "cpr_main.h"
#include "cpr_params.h"
#include "cpr_features.h"
#include "cpr_matrices.h"

class ProcessedPointCloud
{
private:
  Params params;
  pcl::SupervoxelClustering<PointT> super;
  int nbVertices;
  SupervoxelClusters supervoxel_clusters;
  SupervoxelAdjacency supervoxel_adjacency;

public:
  PointCloudT::Ptr cloud;
  ESFDescriptors esf_descriptors;
  EdgeDescriptors edge_descriptors;
  MatrixInt adjacency_matrix;

public:
  ProcessedPointCloud(Params const &p);
  ProcessedPointCloud(char const *metadata_filename);
  //int setParams(int argc, char const *const *argv);
  int loadFile(void);
  int error(void) const;
  void buildGraph(void);
  void buildFeatures(void);
  int build(void);

  pcl::visualization::PCLVisualizer::Ptr visualise(void);
  void addToViewer(pcl::visualization::PCLVisualizer::Ptr viewer);

  //void addSomeColours(pcl::visualization::PCLVisualizer::Ptr viewer);
  void addSomeColours(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<KeyT> nodes);

  int getNbVertices() const;
  std::vector<std::tuple<double,double,double>> exportPointCloud(void) const;
};

#endif /* __DEF_CPR_PROCESSEDPOINTCLOUD_H__ */
