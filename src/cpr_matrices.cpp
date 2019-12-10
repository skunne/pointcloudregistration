
#include <Eigen/Core>
#include <fstream>    // print matrix to file
#include "cpr_loadfiles.h"  // print error message on open file
#include "cpr_features.h"   // distance between two ESF descriptors or two edge descriptors
#include "cpr_matrices.h"

void buildAdjacencyMatrix(SupervoxelAdjacency const &supervoxel_adjacency, Eigen::MatrixXi &adjacency_matrix)
{
  adjacency_matrix = Eigen::MatrixXi::Zero(adjacency_matrix.rows(), adjacency_matrix.cols());
  for (auto edge_itr = supervoxel_adjacency.cbegin(); edge_itr != supervoxel_adjacency.cend(); ++edge_itr)
  {
    adjacency_matrix(edge_itr->first, edge_itr->second) = 1;
    adjacency_matrix(edge_itr->second, edge_itr->first) = 1;
  }
}

// assumes all coeffs are positive!!
void normalizeMatrixTo01(Eigen::MatrixXd &mat)
{
  double coeffMax = 0;
  for (int i = 0; i < mat.rows(); ++i)
    for (int j = 0; j < mat.cols(); ++j)
      coeffMax = (mat(i,j) > coeffMax ? mat(i,j) : coeffMax);

  for (int i = 0; i < mat.rows(); ++i)
    for (int j = 0; j < mat.cols(); ++j)
      mat(i,j) = (mat(i,j)) / coeffMax;
}

VertexSimilarityMatrix::VertexSimilarityMatrix(ESFDescriptors const &source, ESFDescriptors const &dest)
  : m(source.size(), dest.size())
{
  pcl::console::print_highlight("About to build vertex similarity matrix.\n");
  for (auto s_itr = source.cbegin(); s_itr != source.cend(); ++s_itr)
  {
    for (auto d_itr = dest.cbegin(); d_itr != dest.cend(); ++d_itr)
    {
      m(s_itr->first, d_itr->first) = esfDistance(s_itr->second, d_itr->second);
    }
  }
  normalizeMatrixTo01(m);
  pcl::console::print_info("    Successfully built vertex similarity matrix.\n");
}

EdgeSimilarityMatrix::EdgeSimilarityMatrix(EdgeDescriptors const &source, EdgeDescriptors const &dest)
  : m(source.size(), dest.size())
{
  pcl::console::print_highlight("About to build edge similarity matrix.\n");
  unsigned int i = 0;
  for (auto s_itr = source.cbegin(); s_itr != source.cend(); ++s_itr)
  {
    unsigned int j = 0;
    for (auto d_itr = dest.cbegin(); d_itr != dest.cend(); ++d_itr)
    {
      // OF COURSE THIS MEANS WE MUST KEEP AN ORDERED LIST OF THE EDGES SOMEWHERE
      // but that's ok because in c++, std::map is guaranteed to be sorted by key
      m(i,j) = edgeDistance(s_itr->second, d_itr->second);
      ++j;
    }
    ++i;
  }
  normalizeMatrixTo01(m);
  pcl::console::print_info("    Successfully built edge similarity matrix.\n");
}

void printMatrixToFile(char const *filename, Eigen::MatrixXi const &m)
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
void printMatrixToFile(char const *filename, Eigen::MatrixXd const &m)
{
  std::fstream output(filename, std::fstream::out | std::fstream::trunc);

  pcl::console::print_info("    Saving matrix to:\n      ");
  pcl::console::print_info(filename);
  pcl::console::print_info("\n");
  if (!output)
    errorLoadingFile("output", filename);
  else
    output << m << std::endl;
}
