
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
#include "cpr_graphmatching_path.h"
/*
#include "cpr_params.h"
#include "cpr_loadfiles.h"
#include "cpr_clustering.h"
#include "cpr_connectedcomponents.h"
#include "cpr_features.h"
#include "cpr_matrices.h"

#include <Eigen/Dense>    // matrices
*/
//#include <sstream>          // istringstream to parse input for loadGraphMatching()
//#include <string>
//#include "cpr_loadfiles.h"  // errorLoadingFile() for loadGraphMatching()


// int loadGraphMatching(char const *filename, std::vector<KeyT> &mapping)
// {
//   std::ifstream infile(filename);
//   if (!infile)
//     return errorLoadingFile("graph matching", filename);
//   mapping.clear();
//   std::string line;
//   std::getline(infile, line); // skip header
//   while (std::getline(infile, line))
//   {
//       std::istringstream iss(line);
//       int i, u, rank, qcv, rand, path;
//       iss >> i >> u >> rank >> qcv >> rand >> path; // check for error
//       mapping.push_back(qcv - 1); // -1 because the graph matching algo returns indices starting at 1 instead of 0
//       // process pair (a,b)
//   }
//   return 0;
// }

/* add new element in correct position */
// void insert_sorted(std::vector<KeyT> &src, std::vector<KeyT> &dst, int i, int j, VertexSimilarityMatrix const &score)
// {
//   std::vector<KeyT>::iterator src_itr = src.begin();
//   std::vector<KeyT>::iterator dst_itr = dst.begin();
//   while (src_itr != src.end() && score.m(i,j) > score.m(*src_itr, *dst_itr))
//   {
//     ++src_itr;
//     ++dst_itr;
//   }
//   src.insert(src_itr, i);
//   dst.insert(dst_itr, j);
//
//   pcl::console::print_info("insert_sorted: ");
//   for (std::size_t k = 0; k < src.size(); ++k)
//     pcl::console::print_info("%f, ", score.m(src[k], dst[k]));
//   pcl::console::print_info("\n");
// }

// void findSimilarNodes(VertexSimilarityMatrix const &vsim_mat, std::vector<KeyT> const &matching, std::vector<KeyT> &src, std::vector<KeyT> &dst)
// {
//   src.clear();
//   dst.clear();
//   // find 8 smallest (source,dest) node pairs
//   std::size_t i;
//   for (i = 0; i < 8; ++i)
//     insert_sorted(src, dst, i, matching[i], vsim_mat);
//   for (; i < matching.size(); ++i)
//   {
//     if (matching[i] < vsim_mat.m.cols()
//       && vsim_mat.m(i, matching[i]) < vsim_mat.m(src[7], dst[7]))
//       //&& std::find(dst.cbegin(), dst.cend(), matching[i]) == dst.cend())  //avoid duplicates in dst
//     {
//       src.pop_back();
//       dst.pop_back();
//       insert_sorted(src, dst, i, matching[i], vsim_mat);
//     }
//   }
//
//   pcl::console::print_highlight("Scores retenus: \n    ");
//   for (std::size_t i = 0; i < src.size(); ++i)
//     pcl::console::print_info("%f, ", vsim_mat.m(src[i], dst[i]));
//   pcl::console::print_info("\n");
// }

int
main (int argc, char ** argv)
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

  printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

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

  MatrixDouble x(ng, nh);   // matrix to store the graph matching permutation
  x.fill(1.0 / (ng < nh ? ng : nh));  // trivial initial feasible solution
  //x.Constant(ng, nh, 1.0 / static_cast<double>(ng < nh ? ng : nh));
  //x << 0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2,  0.2, 0.2, 0.2, 0.2, 0.2;

  //std::cout << "Initial graph-matching solution:" << std::endl;
  //std::cout << x << std::endl;

  gm.frankWolfe(0.0, &x, &x);

  std::cout << "Final graph-matching solution:" << std::endl;
  std::cout << x << std::endl;

  // TODO find geometric transform corresponding to the matching

  // TODO RanSaC

  // TODO ICP

  return (0);
}
