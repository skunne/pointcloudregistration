//#include <iostream>   // std::cout
//#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

#include "test_frankwolfe.h"  // run_print_compare(), print_matrix_D()

#include "cpr_graphmatching_path.h"
#include "cpr_matrices.h"
#include "cpr_main.h"


double test_two_twonodes_graphs()
{
  int const ng = 2;
  int const nh = 2;

  MatrixInt adj(ng,ng);    // graph = 2 nodes with one edge
  adj << 0, 1,            // second graph = first graph
         1, 0;

  MatrixDouble vsim_mat(ng,nh);  // similarity matrix = 1111 - identity ==> solution to the graph matching problem must be identity
  vsim_mat << 0.95, 0.10,
              0.12, 0.90;

  EdgeDescriptors ed;
  ed[std::make_pair(1,0)] = std::make_tuple(0, 3.14/2.0, 0, 1); // just one edge
  ed[std::make_pair(0,1)] = ed[std::make_pair(1,0)];

  EdgeSimilarityMatrix esim_mat(ed, ed);  // both graphs have the same edge

  print_similarity_matrices(vsim_mat, esim_mat.m);

  print_matrix_D(ng, nh, &vsim_mat, &esim_mat);

  // // human-known graph-matching
  // MatrixDouble human_x(ng, nh);
  // human_x.setIdentity();

  return run_print_compare(ng, nh, &vsim_mat, &esim_mat, &adj, &adj);//, &human_x);
}
