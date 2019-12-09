
#ifndef __DEF_CPR_MATRICES_H__
# define __DEF_CPR_MATRICES_H__

#include <Eigen/Core>
#include "cpr_main.h"

class VertexSimilarityMatrix
{
public:
  Eigen::MatrixXd m;

public:
  VertexSimilarityMatrix(ESFDescriptors const &source, ESFDescriptors const &dest);
};

class EdgeSimilarityMatrix
{
public:
  Eigen::MatrixXd m;

public:
  EdgeSimilarityMatrix(EdgeDescriptors const &source, EdgeDescriptors const &dest);
};

void buildAdjacencyMatrix(SupervoxelAdjacency const &supervoxel_adjacency,
  Eigen::MatrixXi &adjacency_matrix);

void printMatrixToFile(char const *filename, Eigen::MatrixXi const &m);
void printMatrixToFile(char const *filename, Eigen::MatrixXd const &m);

#endif /* __DEF_CPR_MATRICES_H__ */
