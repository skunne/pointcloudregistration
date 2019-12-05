
#ifndef __DEF_CLUSTERING_H__
# define __DEF_CLUSTERING_H__

#include "cpr_main.h"


/*
** clustering.cpp
*/

/*
** change cluster labels to remove holes left by SupervoxelClustering
*/
void renumberVertices(SupervoxelClusters *supervoxel_clusters,
                      SupervoxelAdjacency *supervoxel_adjacency);

/*
** delete clusters of size < 4
*/
int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters,
                          SupervoxelAdjacency &supervoxel_adjacency);

/*
** build a graph from a point cloud
** the last two arguments will receive the output
** supervoxel_clusters: vertices
** supervoxel_adjacency: edges
*/
void performClustering(PointCloudT::Ptr cloud,
  pcl::SupervoxelClustering<PointT> &super,
  struct Params const *params,
  SupervoxelClusters &supervoxel_clusters,
  SupervoxelAdjacency &supervoxel_adjacency);


#endif /* __DEF_CLUSTERING_H__ */
