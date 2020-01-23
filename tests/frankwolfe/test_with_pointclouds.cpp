
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
#include "cpr_graphmatching_path.h"

double run_print_compare(std::size_t ng, std::size_t nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj, MatrixDouble const *humansolution);
void print_similarity_matrices(MatrixDouble const &vsim, MatrixDouble const &esim);
void print_matrix_D(std::size_t ng, std::size_t nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim);

double test_with_pointclouds (int argc, char ** argv)
{
  if (argc < 3)
    return printUsage(argv[0]);

  //Params params(argv[1]);

  ProcessedPointCloud ppc_source(argv[1]);
  ProcessedPointCloud ppc_dest(argv[2]);

  if (ppc_source.error() || ppc_dest.error())
    return 1;

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  //pcl::visualization::PCLVisualizer::Ptr viewer_source =
  //ppc_source.visualise();

  build_error = ppc_dest.build();
  if (build_error)
    return build_error;

  //pcl::visualization::PCLVisualizer::Ptr viewer_dest =
  //ppc_dest.visualise();

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  //printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  //printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

  // pcl::visualization::PCLVisualizer::Ptr viewer =
  //visualisation(ppc_source, ppc_dest);

  pcl::visualization::PCLVisualizer::Ptr viewer_source =
    ppc_source.visualise();
  pcl::visualization::PCLVisualizer::Ptr viewer_dest =
    ppc_dest.visualise();

  /*
  std::vector<KeyT> graph_matching;
  loadGraphMatching("output/matching/big1_big1.match", graph_matching);

  std::vector<KeyT> pointsToColour_source;
  std::vector<KeyT> pointsToColour_dest;

  // select 8 arbitrary points
  for (std::size_t i = 3; i < 83; i += 10)
  {
    pointsToColour_source.push_back(i);
    pointsToColour_dest.push_back(graph_matching[i]);
  }
  //findSimilarNodes(vsim_mat, graph_matching, pointsToColour_source, pointsToColour_dest);

  //ppc_source.addSomeColours(viewer_source, pointsToColour_source);
  //ppc_dest.addSomeColours(viewer_dest, pointsToColour_dest);
  */

  while (!viewer_source->wasStopped())
    viewer_source->spinOnce(100);
  while (!viewer_dest->wasStopped())
    viewer_dest->spinOnce(100);

  //while (!(viewer->wasStopped() && viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //while (!(viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //  ;   // wait for user to close window before halting

  // TODO match graphs
  GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);

  int const ng = ppc_source.getNbVertices();
  int const nh = ppc_dest.getNbVertices();
  int const min_ng_nh = ng < nh ? ng : nh;

  MatrixDouble almost_identity(ng, nh);
  almost_identity.topLeftCorner(min_ng_nh, min_ng_nh).setIdentity();

  return run_print_compare(ng, nh, &vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix, &almost_identity);
}
