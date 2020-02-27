#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
#include "cpr_graphmatching_frankwolfe.h" // if using Frank-Wolfe quadratic programming algorithm
#include "cpr_graphmatching_nonlin.h"     // if using Ipopt nonlinear solver

#include "test_icpransac.h"

// void get_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dest)
// {
//   // Fill in the CloudIn data
//   cloud_source->width    = 5;
//   cloud_source->height   = 1;
//   cloud_source->is_dense = false;
//   cloud_source->points.resize (cloud_source->width * cloud_source->height);
//   for (std::size_t i = 0; i < cloud_source->points.size (); ++i)
//   {
//     cloud_source->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
//     cloud_source->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
//     cloud_source->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
//   }
//   std::cout << "Saved " << cloud_source->points.size () << " data points to input:"
//       << std::endl;
//   for (std::size_t i = 0; i < cloud_source->points.size (); ++i) std::cout << "    " <<
//       cloud_source->points[i].x << " " << cloud_source->points[i].y << " " <<
//       cloud_source->points[i].z << std::endl;
//   *cloud_dest = *cloud_source;
//   std::cout << "size:" << cloud_dest->points.size() << std::endl;
//   for (std::size_t i = 0; i < cloud_source->points.size (); ++i)
//     cloud_dest->points[i].x = cloud_source->points[i].x + 0.7f;
//   std::cout << "Transformed " << cloud_source->points.size () << " data points:"
//       << std::endl;
//   for (std::size_t i = 0; i < cloud_dest->points.size (); ++i)
//     std::cout << "    " << cloud_dest->points[i].x << " " <<
//       cloud_dest->points[i].y << " " << cloud_dest->points[i].z << std::endl;
// }


int
 main (int argc, char** argv)
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dest (new pcl::PointCloud<pcl::PointXYZ>);
  //
  // get_point_clouds(cloud_source, cloud_dest);

  if (argc < 3)
    return printUsage(argv[0]);

  ProcessedPointCloud ppc_source(argv[1]);
  ProcessedPointCloud ppc_dest(argv[2]);

  if (ppc_source.error() || ppc_dest.error())
    return 1;

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  build_error = ppc_dest.build();
  if (build_error)
    return build_error;

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);


  Eigen::Matrix4f icp_transform;
  double icp_score = icpransac(ppc_source.cloud, ppc_dest.cloud, icp_transform);

  std::cout << "ICP/Ransac score: " << icp_score << std::endl;
  std::cout << "ICP/Ransac transform:" << std::endl << icp_transform << std::endl;



 return (0);
}
