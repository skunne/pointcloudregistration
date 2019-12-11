
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
/*
#include "cpr_params.h"
#include "cpr_loadfiles.h"
#include "cpr_clustering.h"
#include "cpr_connectedcomponents.h"
#include "cpr_features.h"
#include "cpr_matrices.h"

#include <Eigen/Dense>    // matrices
*/

int
main (int argc, char ** argv)
{
  if (argc < 3)
    return printUsage(argv[0]);

  //Params params(argv[1]);

  ProcessedPointCloud ppc_source(argv[1]);
  ProcessedPointCloud ppc_dest(argv[2]);

  ppc_source.build();

  //pcl::visualization::PCLVisualizer::Ptr viewer_source =
  ppc_source.visualise();

  ppc_dest.build();

  //pcl::visualization::PCLVisualizer::Ptr viewer_dest =
  ppc_dest.visualise();

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

  // pcl::visualization::PCLVisualizer::Ptr viewer =
  visualisation(ppc_source, ppc_dest);

  //while (!(viewer->wasStopped() && viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //while (!(viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //  ;   // wait for user to close window before halting

  // TODO match graphs

  // TODO find geometric transform corresponding to the matching

  // TODO RanSaC

  // TODO ICP

  return (0);
}
