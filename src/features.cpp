
#include "main.h"
#include <pcl/common/distances.h>

// Calculate edge descriptors according to Huang et al 2017, Fig. 5
void calculate_angles_and_length(PointT const &p1, PointT const &p2, double &angle_x, double &angle_y, double &angle_z, double &length)
{
  angle_x = 0.0;
  angle_y = 0.0;
  angle_z = 0.0;
  length = pcl::euclideanDistance(p1, p2);
}

void calculate_esf_descriptors(SupervoxelClusters const &sv_clusters, ESFDescriptors &esf_descriptors)
{
  pcl::console::print_highlight ("Calculating ESF descriptors\n");

  // map supervoxel_key => esf_descriptor, storing the esf vector for each supervoxel cluster


  pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf_calculator;

  // return variable for ESFEstimation::compute()
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esf_singlepoint_pointcloud;
  //typename ESFEstimation<PointInT, PointOutT>::PointCloudOut esf_singlepoint_pointcloud;
  //pcl::PointCloud<pcl::ESFSignature640> esf_singlepoint_pointcloud;

  // map (supervoxel1_key, supervoxel2_key) => (anglex, angley, anglez, distance)
  // storing info for each pair of adjacent supervoxels

  //foreach key, cluster (k,c) in supervoxel_clusters:
  //    esfdescriptors.add?  (k, ESF(c->voxels_))

  for (SupervoxelClusters::const_iterator sv_itr = sv_clusters.cbegin();
      sv_itr != sv_clusters.cend ();
      ++sv_itr)
  {
    //pcl::console::print_info ("    Supervoxel % 4d    size % 4d\n", sv_itr->first, sv_itr->second->voxels_->size());
    esf_descriptors[sv_itr->first] = ESFHist();
    esf_calculator.setInputCloud(sv_itr->second->voxels_); // feed point cloud corresponding to this supervoxel
    //pcl::console::print_info ("    Local input cloud set\n");
    esf_calculator.compute(esf_singlepoint_pointcloud);
    //pcl::console::print_info ("    Histogram calculated\n");
    //for (std::size_t d = 0; d < esf_singlepoint_pointcloud.points[0].histogram.size (); ++d)
    for (std::size_t d = 0; d < HISTOGRAM_SIZE; ++d)
    {
      //pcl::console::print_info("        Copying bin %d...\n", d);
      //KeyT label = sv_itr->first;
      //pcl::console::print_info("          Got label\n");
      //ESFHist hist = esf_descriptors[label];
      //pcl::console::print_info("          Got hist\n");
      //auto singlepoint = esf_singlepoint_pointcloud[0];
      //pcl::console::print_info("          Got point 0\n");
      //float tobecopied = singlepoint.histogram[d];
      //pcl::console::print_info("          Got bin %d\n", d);
      //hist.push_back(tobecopied);
      //pcl::console::print_info("          Copied!\n");
      esf_descriptors[sv_itr->first].push_back(esf_singlepoint_pointcloud.points[0].histogram[d]);
    }
    //esf_calculator->computeESF(sv_itr->second->voxels_, esf_descriptors[sv_itr->first]);
    //pcl::console::print_info ("    Histogram copied\n");
  }

}

void calculate_edges_descriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, EdgeDescriptors &edge_descriptors)
{
  pcl::console::print_highlight ("Calculating edge descriptors\n");

  //foreach edge between two clusters:
  //    calculer anglex,angley,anglez,longueur
  //To get info for the edges between adjacent supervoxels, we need to iterate through the supervoxel adjacency multimap
  for (SupervoxelAdjacency::const_iterator label_itr = sv_adjacency.cbegin (); label_itr != sv_adjacency.cend (); )
  {
    //First get the label
    KeyT supervoxel1_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel1 = sv_clusters.at (supervoxel1_label);

    //Now we need to iterate through the adjacent supervoxels
    //PointCloudT adjacent_supervoxel_centers;
    for (auto adjacent_itr = sv_adjacency.equal_range (supervoxel1_label).first; adjacent_itr!=sv_adjacency.equal_range (supervoxel1_label).second; ++adjacent_itr)
    {
      KeyT supervoxel2_label = adjacent_itr->second;
      pcl::Supervoxel<PointT>::Ptr supervoxel2 = sv_clusters.at (supervoxel2_label);
      double angle_x;
      double angle_y;
      double angle_z;
      double length;
      calculate_angles_and_length(supervoxel1->centroid_, supervoxel2->centroid_, angle_x, angle_y, angle_z, length);
      edge_descriptors[std::make_pair(supervoxel1_label, supervoxel2_label)] = std::make_tuple(angle_x, angle_y, angle_z, length);
      //adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }//Move iterator forward to next label
    label_itr = sv_adjacency.upper_bound (supervoxel1_label);
  }
}

void calculate_descriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, ESFDescriptors &esf_descriptors, EdgeDescriptors &edge_descriptors)
{
  calculate_esf_descriptors(sv_clusters, esf_descriptors);
  calculate_edges_descriptors(sv_clusters, sv_adjacency, edge_descriptors);
}
