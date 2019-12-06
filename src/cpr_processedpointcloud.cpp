
#include <fstream>
#include "cpr_processedpointcloud.h"

ProcessedPointCloud::ProcessedPointCloud(char const *filename, Params const &p)
  : filename(filename), cloud(new PointCloudT), params(p)
{

}

int ProcessedPointCloud::build()
{
  int error_loading_file = loadFile(); //(argv[1], params.is_pcd, cloud);
  if (error_loading_file)
    return (error_loading_file);

  //////////////////////////////  //////////////////////////////
  ////// Building the graph
  //////////////////////////////  //////////////////////////////

  buildGraph();
  buildFeatures();

  return (0);
}



/*
** int ProcessedPointCloud::loadFile()
** defined in file cpr_loadfiles.cpp
*/

void ProcessedPointCloud::buildGraph()
{
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
  ////// Make adjacency matrix
  //////////////////////////////  //////////////////////////////

  nbVertices = supervoxel_clusters.size();
  adjacency_matrix.resize(nbVertices,nbVertices);
  buildAdjacencyMatrix(supervoxel_adjacency, adjacency_matrix);
  printMatrixToFile("output/adjacencymatrix.txt", adjacency_matrix);
}

void ProcessedPointCloud::buildFeatures()
{
  //////////////////////////////  //////////////////////////////
  ////// ESF and edge descriptors calculation
  //////////////////////////////  //////////////////////////////

  calculateDescriptors(supervoxel_clusters, supervoxel_adjacency, esf_descriptors, edge_descriptors);
  // save features to file
  //writeDescriptorsToCSV(argv[1], esf_descriptors, edge_descriptors);
  //calculate_similarity_matrix(m);
}
