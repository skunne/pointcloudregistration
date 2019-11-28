
#include "main.h"
#include "params.h"

int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency)
{
  int nbDissolvedClusters = 0;
  for (SupervoxelClusters::iterator sv_itr = supervoxel_clusters.begin(); sv_itr != supervoxel_clusters.end(); )
  {
    if (supervoxel_clusters[sv_itr->first]->voxels_->size() < 4)
    {
      // TODO REDISPATCH POINTS AND UPDATE ADJACENCY
      for (SupervoxelAdjacency::iterator edge_itr = supervoxel_adjacency.begin(); edge_itr != supervoxel_adjacency.end(); )
      {
        if (edge_itr->first == sv_itr->first || edge_itr->second == sv_itr->first)
          edge_itr = supervoxel_adjacency.erase(edge_itr);
        else
          edge_itr++;
      }
      nbDissolvedClusters++;
      sv_itr = supervoxel_clusters.erase(sv_itr);
    }
    else
    {
      sv_itr++;
    }
  }
  return nbDissolvedClusters;
}

void perform_clustering(PointCloudT::Ptr cloud, pcl::SupervoxelClustering<PointT> &super, struct Params const *params, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency)
{
  if (params->disable_transform)
    super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (params->color_importance);
  super.setSpatialImportance (params->spatial_importance);
  super.setNormalImportance (params->normal_importance);

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("    Found %d supervoxels\n", supervoxel_clusters.size ());
  pcl::console::print_highlight ("Getting supervoxel adjacency\n");

  super.getSupervoxelAdjacency (supervoxel_adjacency);

  //////////////////////////////  //////////////////////////////
  ////// Getting rid of clusters of size 1 or 2 (ESF descriptor requires >= 3 points per cloud)
  //////////////////////////////  //////////////////////////////

  pcl::console::print_highlight ("Dissolving small supervoxels\n");
  int nbDissolvedClusters = dissolveSmallClusters(supervoxel_clusters, supervoxel_adjacency);
  pcl::console::print_info ("    Dissolved %d supervoxels. Left with %d supervoxels\n", nbDissolvedClusters, supervoxel_clusters.size ());
}
