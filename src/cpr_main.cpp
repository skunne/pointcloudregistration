
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

/* add new element in correct position */
void insert_sorted(std::vector<int> &src, std::vector<int> &dst, int i, int j, VertexSimilarityMatrix const &score)
{
  std::vector<int>::iterator src_itr = src.begin();
  std::vector<int>::iterator dst_itr = dst.begin();
  while (src_itr != src.end() && score.m(i,j) > score.m(*src_itr, *dst_itr))
  {
    ++src_itr;
    ++dst_itr;
  }
  src.insert(src_itr, i);
  dst.insert(dst_itr, j);

  pcl::console::print_info("insert_sorted: ");
  for (std::size_t k = 0; k < src.size(); ++k)
    pcl::console::print_info("%f, ", score.m(src[k], dst[k]));
  pcl::console::print_info("\n");
}

void findSimilarNodes(VertexSimilarityMatrix const &vsim_mat, std::vector<int> &src, std::vector<int> &dst)
{
  /*
  ** TODO: make sure the correspondance is one-to-one!!!
  */

  // initialise vectors with first 10 pairs
  int nbInserted = 0;
  for (int j = 0; nbInserted < 8 && j < vsim_mat.m.cols(); ++j)
  {
    if (vsim_mat.m(0,j) > 0)
    {
      insert_sorted(src, dst, 0, j, vsim_mat);
      nbInserted++;
    }
  }
  pcl::console::print_info("Initialised with %d nodes\n", nbInserted);

  // browse sim matrix and keep best 10 pairs
  // a pair is better if it has a low score in vsim_mat
  // but pairs with score 0 are left aside because they usually correspond to nodes with two dummy ESF
  for (int i = 0; i < vsim_mat.m.rows(); ++i)
    for (int j = 0; j < vsim_mat.m.cols(); ++j)
    {
      if (vsim_mat.m(i, j) > 0 && vsim_mat.m(i, j) < vsim_mat.m(src[7], dst[7]))
      {
        src.pop_back();
        dst.pop_back();
        insert_sorted(src, dst, i, j, vsim_mat);
      }
    }

  pcl::console::print_highlight("Scores retenus: \n    ");
  for (std::size_t i = 0; i < src.size(); ++i)
    pcl::console::print_info("%f, ", vsim_mat.m(src[i], dst[i]));
  pcl::console::print_info("\n");
}

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
  //ppc_source.visualise();

  ppc_dest.build();

  //pcl::visualization::PCLVisualizer::Ptr viewer_dest =
  //ppc_dest.visualise();

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

  // pcl::visualization::PCLVisualizer::Ptr viewer =
  //visualisation(ppc_source, ppc_dest);

  pcl::visualization::PCLVisualizer::Ptr viewer_source =
    ppc_source.visualise();
  pcl::visualization::PCLVisualizer::Ptr viewer_dest =
    ppc_dest.visualise();

  std::vector<int> pointsToColour_source;
  std::vector<int> pointsToColour_dest;

  findSimilarNodes(vsim_mat, pointsToColour_source, pointsToColour_dest);

  ppc_source.addSomeColours(viewer_source, pointsToColour_source);
  ppc_dest.addSomeColours(viewer_dest, pointsToColour_dest);

  while (!viewer_source->wasStopped())
    viewer_source->spinOnce(100);
  while (!viewer_dest->wasStopped())
    viewer_dest->spinOnce(100);

  //while (!(viewer->wasStopped() && viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //while (!(viewer_source->wasStopped() && viewer_dest->wasStopped()))
  //  ;   // wait for user to close window before halting

  // TODO match graphs

  // TODO find geometric transform corresponding to the matching

  // TODO RanSaC

  // TODO ICP

  return (0);
}
