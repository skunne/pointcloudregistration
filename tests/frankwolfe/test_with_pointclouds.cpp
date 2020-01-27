
#include <Eigen/Core>

#include "test_frankwolfe.h"
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
#include "cpr_graphmatching_path.h"

double test_with_pointclouds (int argc, char ** argv)
{
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

  GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);

  int const ng = ppc_source.getNbVertices();
  int const nh = ppc_dest.getNbVertices();

  // MatrixDouble almost_identity(ng, nh);
  // if (ng == nh)
  // {
  //   almost_identity.setIdentity();
  // }
  // else if (ng < nh)
  // {
  //   almost_identity.topLeftCorner(ng, ng).setIdentity();
  //   almost_identity.topRightCorner(ng, nh-ng).setZero();
  // }
  // else // nh < ng
  // {
  //   almost_identity.topLeftCorner(nh, nh).setIdentity();
  //   almost_identity.bottomLeftCorner(ng-nh, nh).setZero();
  // }
  //
  // std::cout << almost_identity;

  double result =
    run_print_compare(ng, nh, &vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);//, &almost_identity);

  // pcl::visualization::PCLVisualizer::Ptr viewer_source =
  //   ppc_source.visualise();
  // pcl::visualization::PCLVisualizer::Ptr viewer_dest =
  //   ppc_dest.visualise();
  //
  // while (!viewer_source->wasStopped())
  //   viewer_source->spinOnce(100);
  // while (!viewer_dest->wasStopped())
  //   viewer_dest->spinOnce(100);

  return result;
}
