
#include "main.h"
#include "params.h"


int
main (int argc, char ** argv)
{
  struct Params params;
  int syntax_error = set_params(argc, argv, &params);
  if (syntax_error)
    return (syntax_error);

  PointCloudT::Ptr cloud (new PointCloudT);
  int error_loading_file = load_file(argv[1], params.is_pcd, cloud);
  if (error_loading_file)
    return (error_loading_file);

  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  SupervoxelClusters supervoxel_clusters;
  SupervoxelAdjacency supervoxel_adjacency;
  pcl::SupervoxelClustering<PointT> super (params.voxel_resolution, params.seed_resolution);
  perform_clustering(cloud, super, &params, supervoxel_clusters, supervoxel_adjacency);

  //////////////////////////////  //////////////////////////////
  ////// ESF and edge descriptors calculation
  //////////////////////////////  //////////////////////////////

  ESFDescriptors esf_descriptors;
  EdgeDescriptors edge_descriptors;
  //SimilarityMatrix m;
  calculate_descriptors(supervoxel_clusters, supervoxel_adjacency, esf_descriptors, edge_descriptors);
  //calculate_similarity_matrix(m);

  //////////////////////////////  //////////////////////////////
  ////// This is the visualisation part
  //////////////////////////////  //////////////////////////////

  visualisation(super, supervoxel_clusters, supervoxel_adjacency);
  return (0);
}
