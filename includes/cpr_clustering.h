
#ifndef __DEF_CLUSTERING_H__
# define __DEF_CLUSTERING_H__

#include "cpr_main.h"

int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);
void performClustering(PointCloudT::Ptr cloud, pcl::SupervoxelClustering<PointT> &super, struct Params const *params, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency);


#endif /* __DEF_CLUSTERING_H__ */
