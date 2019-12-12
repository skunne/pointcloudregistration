
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
#include <sstream>
#include <string>
#include "cpr_loadfiles.h"


int loadGraphMatching(char const *filename, std::vector<KeyT> &mapping)
{
  std::ifstream infile(filename);
  if (!infile)
    return errorLoadingFile("graph matching", filename);
  mapping.clear();
  std::string line;
  std::getline(infile, line); // skip header
  while (std::getline(infile, line))
  {
      std::istringstream iss(line);
      int i, u, rank, qcv, rand, path;
      iss >> i >> u >> rank >> qcv >> rand >> path; // check for error
      mapping.push_back(path - 1); // -1 because the graph matching algo returns indices starting at 1 instead of 0
      // process pair (a,b)
  }
  return 0;
}

/* add new element in correct position */
void insert_sorted(std::vector<KeyT> &src, std::vector<KeyT> &dst, int i, int j, VertexSimilarityMatrix const &score)
{
  std::vector<KeyT>::iterator src_itr = src.begin();
  std::vector<KeyT>::iterator dst_itr = dst.begin();
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

void findSimilarNodes(VertexSimilarityMatrix const &vsim_mat, std::vector<KeyT> &src, std::vector<KeyT> &dst)
{
  // find prefered destination node for every source node
  std::vector<KeyT> preferedDest;
  preferedDest.reserve(vsim_mat.m.rows());
  for (Eigen::Index i = 0; i < vsim_mat.m.rows(); ++i)
  {
    preferedDest.push_back(0);
    for (Eigen::Index j = 1; j < vsim_mat.m.cols(); ++j)
    {
      if (vsim_mat.m(i, j) > 0 && ((vsim_mat.m(i, j) < vsim_mat.m(i, preferedDest[i])) || vsim_mat.m(i, preferedDest[i]) == 0))
        preferedDest[i] = j;
    }
  }

  pcl::console::print_info("preferedDest size: %d\n", preferedDest.size());

  src.clear();
  dst.clear();
  // find 8 smallest (source,dest) node pairs
  std::size_t i;
  for (i = 0; i < 8; ++i)
    insert_sorted(src, dst, i, preferedDest[i], vsim_mat);
  for (; i < preferedDest.size(); ++i)
  {
    if (vsim_mat.m(i, preferedDest[i]) < vsim_mat.m(src[7], dst[7])
      && std::find(dst.cbegin(), dst.cend(), preferedDest[i]) == dst.cend())  //avoid duplicates in dst
    {
      src.pop_back();
      dst.pop_back();
      insert_sorted(src, dst, i, preferedDest[i], vsim_mat);
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

  std::vector<KeyT> graph_matching;
  loadGraphMatching("output/matching/big1_rot.match", graph_matching);

  std::vector<KeyT> pointsToColour_source;
  std::vector<KeyT> pointsToColour_dest;

  for (std::size_t i = 0; i < 80; i += 10)
  {
    pointsToColour_source.push_back(i);
    pointsToColour_dest.push_back(graph_matching[i]);
  }
  //findSimilarNodes(vsim_mat, pointsToColour_source, pointsToColour_dest);

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
