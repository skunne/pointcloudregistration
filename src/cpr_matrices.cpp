
#include <Eigen/Core>
#include <fstream>    // print matrix to file
#include "cpr_loadfiles.h"  // print error message on open file
#include "cpr_features.h"   // distance between two ESF descriptors or two edge descriptors
#include "cpr_matrices.h"

void buildAdjacencyMatrix(SupervoxelAdjacency const &supervoxel_adjacency, MatrixInt &adjacency_matrix)
{
  adjacency_matrix = MatrixInt::Zero(adjacency_matrix.rows(), adjacency_matrix.cols());
  for (auto edge_itr = supervoxel_adjacency.cbegin(); edge_itr != supervoxel_adjacency.cend(); ++edge_itr)
  {
    adjacency_matrix(edge_itr->first, edge_itr->second) = 1;
    adjacency_matrix(edge_itr->second, edge_itr->first) = 1;
  }
}

// assumes all coeffs are positive!!
void normalizeMatrixTo01(MatrixDouble &mat)
{
  double coeffMax = 0;
  for (int i = 0; i < mat.rows(); ++i)
    for (int j = 0; j < mat.cols(); ++j)
      coeffMax = (mat(i,j) > coeffMax ? mat(i,j) : coeffMax);

  if (coeffMax > 0)
  {
    for (int i = 0; i < mat.rows(); ++i)
      for (int j = 0; j < mat.cols(); ++j)
        //mat(i,j) = (mat(i,j)) / coeffMax;
        mat(i,j) = 1.0 - (mat(i,j)) / coeffMax;   // reverse 0 and 1 because this is similarity matrix, not distance matrix
  }
}

VertexSimilarityMatrix::VertexSimilarityMatrix(ESFDescriptors const &source, ESFDescriptors const &dest)
  : m(source.size(), dest.size())
{
  pcl::console::print_highlight("Building vertex similarity matrix.\n");
  for (auto s_itr = source.cbegin(); s_itr != source.cend(); ++s_itr)
    for (auto d_itr = dest.cbegin(); d_itr != dest.cend(); ++d_itr)
      m(s_itr->first, d_itr->first) = esfDistance(s_itr->second, d_itr->second);
  normalizeMatrixTo01(m);
  pcl::console::print_info("    Successfully built %lu,%lu vertex similarity matrix.\n", source.size(), dest.size());
}

EdgeSimilarityMatrix::EdgeSimilarityMatrix(EdgeDescriptors const &source, EdgeDescriptors const &dest)
  : m(source.size(), dest.size())
{
  pcl::console::print_highlight("Building edge similarity matrix.\n");
  //sourceEdgeIndex.reserve(source.size());  //std::map::reserve() does not exist
  //destEdgeIndex.reserve(dest.size());
  unsigned int i = 0;
  for (auto s_itr = source.cbegin(); s_itr != source.cend(); ++s_itr)
  {
    sourceEdgeIndex[s_itr->first] = i;
    unsigned int j = 0;
    for (auto d_itr = dest.cbegin(); d_itr != dest.cend(); ++d_itr)
    {
      destEdgeIndex[d_itr->first] = j;
      m(i,j) = edgeDistance(s_itr->second, d_itr->second);
      //std::cout << "EdgeSimilarityMatrix()  m(" << i << ',' << j << ") = " << m(i,j) << std::endl;
      ++j;
    }
    ++i;
  }
  //std::cout << "EdgeSimilarityMatrix before normalisation:" << std::endl;
  //std::cout << m << std::endl;
  normalizeMatrixTo01(m);
  //std::cout << "EdgeSimilarityMatrix after normalisation:" << std::endl;
  //std::cout << m << std::endl;
  pcl::console::print_info("    Successfully built %lu,%lu edge similarity matrix.\n", source.size(), dest.size());
}

// artificial constructor for debug purposes
EdgeSimilarityMatrix::EdgeSimilarityMatrix(
  std::map<std::pair<KeyT, KeyT>, unsigned int> const &srcEIndex,
  std::map<std::pair<KeyT, KeyT>, unsigned int> const &dstEIndex,
  MatrixDouble const &mat)
  : sourceEdgeIndex(srcEIndex), destEdgeIndex(dstEIndex), m(mat)
{
}

// artificial constructor for debug purposes
VertexSimilarityMatrix::VertexSimilarityMatrix(MatrixDouble const &mat)
  : m(mat)
{
}

void printMatrixToFile(char const *filename, MatrixInt const &m)
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
void printMatrixToFile(char const *filename, MatrixDouble const &m)
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

template<class T>
void printVectorAsMatrix(std::vector<T> const &v, std::size_t height, std::size_t width)
{
  for (std::size_t row = 0; row < height; ++row)
  {
    for (std::size_t col = 0; col < width; ++col)
      std::cout << v[row * width + col] << ' ';
    std::cout << std::endl;
  }
}
