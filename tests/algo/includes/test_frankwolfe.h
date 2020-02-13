#ifndef __DEF_TEST_FRANKWOLFE_H__
# define __DEF_TEST_FRANKWOLFE_H__

#include "cpr_matrices.h"  // EdgeSimilarityMatrix
#include "cpr_typedef.h"   // typedefs MatrixDouble, EdgeSimilarityMatrix, MatrixInt

// printandcomparesolution.cpp
double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj);//, MatrixDouble const *humansolution);

void print_similarity_matrices(MatrixDouble const &vsim, MatrixDouble const &esim);

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble const *humansolution, char const *human_solution_name,
  MatrixDouble *return_solution);

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble const *humansolution, char const *human_solution_name);

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble *return_solution);

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj);

void print_matrix_D(std::size_t ng, std::size_t nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim);

// test_with_pointclouds.cpp
//double test_with_pointclouds (int argc, char ** argv);
double test_with_pointclouds (int argc, char const *const *argv);

#endif /* __DEF_TEST_FRANKWOLFE_H__ */
