
#include <fstream>
#include "cpr_loadfiles.h"
#include "cpr_clustering.h"
#include "cpr_connectedcomponents.h"
#include "cpr_visualisation.h"
#include "cpr_processedpointcloud.h"

#include "cpr_debug_supervoxel.h"

ProcessedPointCloud::ProcessedPointCloud(Params const &p)
  : params(p), cloud(new PointCloudT), super (params.voxel_resolution, params.seed_resolution)
{
}

ProcessedPointCloud::ProcessedPointCloud(char const *metadata_filename)
  : params(metadata_filename), cloud(new PointCloudT), super (params.voxel_resolution, params.seed_resolution)
{
}

int ProcessedPointCloud::error(void) const
{
  return params.error;
}

int ProcessedPointCloud::build(void)
{
  int error_loading_file = loadFile(); //(argv[1], params.is_pcd, cloud);
  if (error_loading_file)
    return (error_loading_file);

  //cprdbg::supervoxel::print_pointcloud(cloud, 2);


  buildGraph();

  buildFeatures();

  return (0);
}



/*
** int ProcessedPointCloud::loadFile()
** defined in file cpr_loadfiles.cpp
*/

void ProcessedPointCloud::buildGraph(void)
{
  //pcl::SupervoxelClustering<PointT> super (params.voxel_resolution, params.seed_resolution);
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
  //printMatrixToFile(params.adjacency_filename.c_str(), adjacency_matrix);
}

void ProcessedPointCloud::buildFeatures(void)
{
  calculateDescriptors(supervoxel_clusters, supervoxel_adjacency, params.voxel_resolution, esf_descriptors, edge_descriptors);
  // save features to file
  //writeDescriptorsToCSV(argv[1], esf_descriptors, edge_descriptors);
  //calculate_similarity_matrix(m);
}

//pcl::visualization::PCLVisualizer::Ptr
// pcl::visualization::PCLVisualizer::Ptr ProcessedPointCloud::visualise(void)
// {
//   // maybe encapsulate this in Boost::Thread or call fork() ?
//   pcl::console::print_highlight ("Initialising visualisation\n");
//
//   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//
//   addToViewer(viewer);
//   //addSomeColours(viewer);
//
//   //while (!viewer->wasStopped ())
//   //{
//   //  viewer->spinOnce (100);
//   //}
//   return viewer;
// }

int ProcessedPointCloud::getNbVertices(void) const
{
  return nbVertices;
}

std::vector<std::tuple<double,double,double>> ProcessedPointCloud::exportCentroidPointCloud(void) const
{
  std::vector<std::tuple<double,double,double>> out;
  for (auto const &p : supervoxel_clusters)
  {
    out.push_back(std::make_tuple<double,double,double>(p.second->centroid_.x, p.second->centroid_.y, p.second->centroid_.z));
  }
  return out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ProcessedPointCloud::exportCentroidPointCloud(void) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr out(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto const &p : supervoxel_clusters)
  {
    out.push_back(pcl::PointXYZ(p.second->centroid_.x, p.second->centroid_.y, p.second->centroid_.z));
  }
  return out;
}


float ProcessedPointCloud::get_seed_resolution(void) const
{
  return params.get_seed_resolution();
}
float ProcessedPointCloud::get_voxel_resolution(void) const
{
  return params.get_voxel_resolution();
}
//
// std::vector<std::vector<std::tuple<double,double,double>>> ProcessedPointCloud::exportClusters(void) const
// {
//   std::vector<std::vector<std::tuple<double,double,double>>> out;
//   for (auto const &p : supervoxel_clusters)
//   {
//     std::vector<std::tuple<double,double,double>> cluster = p.second->voxels_
//     out.push_back(cluster);
//     out.push_back(std::make_tuple<double,double,double>(p.second->centroid_.x, p.second->centroid_.y, p.second->centroid_.z));
//   }
//   return out;
// }
