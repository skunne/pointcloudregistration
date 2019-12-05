
#ifndef __DEF_CPR_MATRICES_H__
# define __DEF_CPR_MATRICES_H__
#include "cpr_main.h"


void buildAdjacencyMatrix(SupervoxelAdjacency const &supervoxel_adjacency,
  Eigen::MatrixXd &adjacency_matrix);

void printMatrixToFile(char const *filename, Eigen::MatrixXd m);

#endif /* __DEF_CPR_MATRICES_H__ */
