
#include "cpr_main.h"
#include "cpr_params.h"
#include "cpr_loadfiles.h"
#include "cpr_clustering.h"
#include "cpr_connectedcomponents.h"
#include "cpr_features.h"
#include "cpr_matrices.h"

#include <Eigen/Dense>    // matrices


int
main (int argc, char ** argv)
{
  struct Params params;
  int syntax_error = setParams(argc, argv, &params);
  if (syntax_error)
    return (syntax_error);

  PointCloudT::Ptr cloud (new PointCloudT);
  int error_loading_file = loadFile(argv[1], params.is_pcd, cloud);
  if (error_loading_file)
    return (error_loading_file);

  //////////////////////////////  //////////////////////////////
  ////// Building the graph
  //////////////////////////////  //////////////////////////////

  SupervoxelClusters supervoxel_clusters;
  SupervoxelAdjacency supervoxel_adjacency;
  pcl::SupervoxelClustering<PointT> super (params.voxel_resolution, params.seed_resolution);
  performClustering(cloud, super, &params, supervoxel_clusters, supervoxel_adjacency);

  pcl::console::print_highlight("Getting connected components\n");
  std::vector<std::vector<KeyT>> cc_list;
  //std::map<KeyT, std::size_t> cc_membership;
  int nbCC = getConnectedComponents(supervoxel_clusters, supervoxel_adjacency,
                            cc_list);//, cc_membership);
  pcl::console::print_info("    Got %d connected components\n", nbCC);
  makeGraphConnected(supervoxel_clusters, supervoxel_adjacency,
                            cc_list);//, cc_membership);
  pcl::console::print_info("    Graph is now connected\n");

  //////////////////////////////  //////////////////////////////
  ////// ESF and edge descriptors calculation
  //////////////////////////////  //////////////////////////////

  ESFDescriptors esf_descriptors;
  EdgeDescriptors edge_descriptors;
  //SimilarityMatrix m;
  calculateDescriptors(supervoxel_clusters, supervoxel_adjacency, esf_descriptors, edge_descriptors);

  // save features to file
  //writeDescriptorsToCSV(argv[1], esf_descriptors, edge_descriptors);
  //calculate_similarity_matrix(m);


  //////////////////////////////  //////////////////////////////
  ////// Make adjacency matrix
  //////////////////////////////  //////////////////////////////

  int nbVertices = supervoxel_clusters.size();
  Eigen::MatrixXd adjacency_matrix(nbVertices,nbVertices);
  buildAdjacencyMatrix(supervoxel_adjacency, adjacency_matrix);
  printMatrixToFile("output/adjacencymatrix.txt", adjacency_matrix);



  //////////////////////////////  //////////////////////////////
  ////// This is the visualisation part
  //////////////////////////////  //////////////////////////////

  visualisation(super, supervoxel_clusters, supervoxel_adjacency);
  return (0);
}
