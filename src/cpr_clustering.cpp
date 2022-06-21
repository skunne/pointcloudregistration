
#include "cpr_main.h"
#include "cpr_params.h"
#include "cpr_clustering.h"

void renumberVertices(SupervoxelClusters *supervoxel_clusters, SupervoxelAdjacency *supervoxel_adjacency)
{
  SupervoxelClusters new_supervoxel_clusters;
  SupervoxelAdjacency new_supervoxel_adjacency;
  std::map<KeyT, KeyT>  newIndexFromOld;
  KeyT i = 0;
  for (auto sv_itr = supervoxel_clusters->cbegin();
      sv_itr != supervoxel_clusters->cend(); ++sv_itr)
  {
    new_supervoxel_clusters[i] = sv_itr->second;
    newIndexFromOld[sv_itr->first] = i;
    ++i;
  }
  for (auto edge_itr = supervoxel_adjacency->cbegin();
      edge_itr != supervoxel_adjacency->cend(); ++edge_itr)
  {
    new_supervoxel_adjacency.insert(std::make_pair(newIndexFromOld[edge_itr->first], newIndexFromOld[edge_itr->second]));
  }

  supervoxel_clusters->clear();
  supervoxel_adjacency->clear();
  *supervoxel_clusters = new_supervoxel_clusters;
  *supervoxel_adjacency = new_supervoxel_adjacency;
}

/*int dissolveSmallClusters(SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency)
{
  int nbDissolvedClusters = 0;
  int nbPointsTotal = 0;
  for (SupervoxelClusters::iterator sv_itr = supervoxel_clusters.begin();
      sv_itr != supervoxel_clusters.end(); )
  {
    nbPointsTotal += supervoxel_clusters[sv_itr->first]->voxels_->size();
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
  pcl::console::print_info("    (Total number of points in the clusters: %d)\n", nbPointsTotal);
  return nbDissolvedClusters;
}*/

void performClustering(PointCloudT::Ptr cloud, pcl::SupervoxelClustering<PointT> &super, struct Params const *params, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency &supervoxel_adjacency)
{
  //if (params->disable_transform)
  super.setUseSingleCameraTransform (false);
  super.setInputCloud (cloud);
  super.setColorImportance (params->color_importance);
  super.setSpatialImportance (params->spatial_importance);
  super.setNormalImportance (params->normal_importance);

  pcl::console::print_highlight ("Extracting supervoxels!\n");
  pcl::console::print_info("    Resolution:\n"
                          "        voxel %.1f, seed %.1f\n"
                          "    Weights:\n"
                          "        color %.1f, spatial %.1f, normal %.1f\n",
                          params->voxel_resolution, params->seed_resolution,
                          params->color_importance, params->spatial_importance, params->normal_importance);
  super.extract (supervoxel_clusters);
  pcl::console::print_info ("    Found %d supervoxels\n", supervoxel_clusters.size ());
  //super.refineSupervoxels(5, supervoxel_clusters);
  //pcl::console::print_info ("    Refined supervoxels\n", supervoxel_clusters.size ());
  pcl::console::print_highlight ("Getting supervoxel adjacency\n");
  super.getSupervoxelAdjacency (supervoxel_adjacency);
  pcl::console::print_info ("    Found %d edges\n", supervoxel_adjacency.size() / 2);
  

  //////////////////////////////  //////////////////////////////
  ////// Reindex vertices to remove holes introduced by SupervoxelClustering
  //////////////////////////////  //////////////////////////////
  renumberVertices(&supervoxel_clusters, &supervoxel_adjacency);


  //for (auto edge_itr = supervoxel_adjacency.begin(); edge_itr != supervoxel_adjacency.end(); edge_itr++)
  //  pcl::console::print_info("        %d,%d\n", edge_itr->first, edge_itr->second);


  //////////////////////////////  //////////////////////////////
  ////// Getting rid of clusters of size 1, 2 or 3(ESF descriptor requires > 3 points per cloud)
  //////////////////////////////  //////////////////////////////

  //pcl::console::print_highlight ("Dissolving small supervoxels\n");
  //int nbDissolvedClusters = dissolveSmallClusters(supervoxel_clusters, supervoxel_adjacency);
  //pcl::console::print_info ("    Dissolved %d supervoxels. Left with %d supervoxels\n", nbDissolvedClusters, supervoxel_clusters.size ());
}
