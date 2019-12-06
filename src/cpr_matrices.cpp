

#include <fstream>    // print matrix to file
#include "cpr_loadfiles.h"  // print error message on open file
#include "cpr_matrices.h"

void buildAdjacencyMatrix(SupervoxelAdjacency const &supervoxel_adjacency, Eigen::MatrixXd &adjacency_matrix)
{
  adjacency_matrix = Eigen::MatrixXd::Zero(adjacency_matrix.rows(), adjacency_matrix.cols());
  for (auto edge_itr = supervoxel_adjacency.cbegin(); edge_itr != supervoxel_adjacency.cend(); ++edge_itr)
  {
    adjacency_matrix(edge_itr->first, edge_itr->second) = 1;
    adjacency_matrix(edge_itr->second, edge_itr->first) = 1;
  }
}

VertexSimilarityMatrix::VertexSimilarityMatrix(ESFDescriptors const &source, ESFDescriptors const &dest)
  : m(source.size(), dest.size())
{
}

EdgeSimilarityMatrix::EdgeSimilarityMatrix(EdgeDescriptors const &source, EdgeDescriptors const &dest)
  : m(source.size(), dest.size())
{
}

void printMatrixToFile(char const *filename, Eigen::MatrixXd m)
{
  std::fstream output(filename, std::fstream::out | std::fstream::trunc);

  pcl::console::print_info("    Saving adjacency matrix to:\n      ");
  pcl::console::print_info(filename);
  pcl::console::print_info("\n");
  if (!output)
    errorLoadingFile("output", filename);
  else
    output << m << std::endl;
}
