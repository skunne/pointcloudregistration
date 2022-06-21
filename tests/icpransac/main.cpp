#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include "cpr_processedpointcloud.h"
//#include "cpr_matrices.h"
//#include "cpr_visualisation.h"
//#include "cpr_graphmatching_frankwolfe.h" // if using Frank-Wolfe quadratic programming algorithm
//#include "cpr_graphmatching_nonlin.h"     // if using Ipopt nonlinear solver
#include "cpr_loadfiles.h"    // loadPCDFile

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

int test_printUsage(char const *str)
{
    std::cout << "USAGE:" << std::endl << std::endl;
    std::cout << "  " << str << " <cloudsource.pcd> <clouddest.pcd> <transform.csv>" << std::endl;
    std::cout << "    Apply ICP/Ransac to two point clouds to find" << std::endl;
    std::cout << "    the rigid transform from source to dest." << std::endl;
    std::cout << "    The resulting 4x4 matrix is written to transform.csv." << std::endl;
    return 1;
}

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dest (new pcl::PointCloud<pcl::PointXYZRGBA>);
  //
  // get_point_clouds(cloud_source, cloud_dest);

  if (argc < 3)
    return test_printUsage(argv[0]);

  int error = loadPCDFile(argv[1], cloud_source);
  if (error)
    return error;
  error = loadPCDFile(argv[2], cloud_dest);
  if (error)
    return error;

  // ProcessedPointCloud ppc_source(argv[1]);
  // ProcessedPointCloud ppc_dest(argv[2]);
  //
  // if (ppc_source.error() || ppc_dest.error())
  //   return 1;
  //
  // int build_error = ppc_source.build();
  // if (build_error)
  //   return build_error;
  //
  // build_error = ppc_dest.build();
  // if (build_error)
  //   return build_error;
  //
  // VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  // EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);


  Eigen::Matrix4f icp_transform;
  //double icp_score = icpransac(cloud_source, cloud_dest, icp_transform);
  icpransac(cloud_source, cloud_dest, icp_transform);

  const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "\n");

  std::ofstream outf(argv[3]);

  std::cout << "Writing transform to " << argv[3] << std::endl;
  outf << icp_transform.format(CSVFormat) << std::endl;
  //
  // //std::cout << "ICP/Ransac score: " << icp_score << std::endl;
  // std::cout << "ICP/Ransac transform:" << std::endl << icp_transform << std::endl;
  //
  // GraphMatchingNonlin gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  //
  // gm.run();
  //
  // std::cout << "Final graph-matching solution:" << std::endl;
  // std::cout << gm.matching << std::endl;

 return (0);
}
