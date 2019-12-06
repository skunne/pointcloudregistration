
#ifndef __DEF_CPR_MATRICES_H__
# define __DEF_CPR_MATRICES_H__
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
  Eigen::MatrixXd &adjacency_matrix);

void printMatrixToFile(char const *filename, Eigen::MatrixXd m);

#endif /* __DEF_CPR_MATRICES_H__ */
