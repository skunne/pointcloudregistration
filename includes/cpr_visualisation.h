
#ifndef __DEF_CPR_VISUALISATION_H__
# define __DEF_CPR_VISUALISATION_H__
#include "cpr_main.h"

/*
** visualisation.cpp
*/
void visualisation(pcl::SupervoxelClustering<PointT> const &super, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency const &supervoxel_adjacency);
void addSupervoxelConnectionsToViewer (PointT const &supervoxel_center,
                                       PointCloudT const &adjacent_supervoxel_centers,
                                       std::string supervoxel_name,
                                       pcl::visualization::PCLVisualizer::Ptr &viewer);

#endif /* __DEF_CPR_VISUALISATION_H__ */
