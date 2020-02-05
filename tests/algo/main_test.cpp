#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>

#include "cpr_params.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_graphmatching_path.h"

#include "test_frankwolfe.h"

int test_printUsage(char const *cmd)
{
  std::cout << "SYNOPSIS" << std::endl << std::endl;
  std::cout << cmd << "<meta> <pc0> <pc1> [<pc2> ... <pcn>]" << std::endl;
  std::cout << "    Use the algorithm to register <pc0> against <pc1>, <pc2>, ..., <pcn>" << std::endl;
  std::cout << "    <meta> must be the name of a metadata file. The point cloud filename given inside <meta> is ignored; only the parameters are used." << std::endl;
  return 1;
}

void test_writeresult(ProcessedPointCloud const &ppc_source, ProcessedPointCloud const &ppc_dest, MatrixDouble const &permutation_matrix)
{
  std::vector<std::tuple<double,double,double>> pc_source = ppc_source.exportPointCloud();
  std::vector<std::tuple<double,double,double>> pc_dest = ppc_dest.exportPointCloud();
  
}

int main(int argc, char ** argv)
{
  if (argc <= 3)
    return test_printUsage(argv[0]);

//  std::vector<char const *> names;
//  std::vector<double> results;

  Params params_source(argv[1]);
  params_source.filename = argv[2];
  assert(params_source.error == 0);

  ProcessedPointCloud ppc_source(params_source);
  assert(ppc_source.error() == 0);
  int source_build_error = ppc_source.build();
  assert(source_build_error == 0);

  Params params_dest = params_source;
  params_dest.adjacency_filename = "dest.adj";
  for (int i = 3; i < argc; ++i)
  {
    params_dest.filename = argv[i];
    ProcessedPointCloud ppc_dest(params_dest);
    assert(ppc_dest.error() == 0);
    int dest_build_error = ppc_dest.build();
    assert(dest_build_error == 0);

    VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
    EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

    int const n_source = ppc_source.getNbVertices();
    int const n_dest = ppc_dest.getNbVertices();

    MatrixDouble permutation_matrix(n_source, n_dest);
    permutation_matrix.fill(1.0 / (n_source < n_dest ? n_dest : n_source));
    GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
    gm.frankWolfe(0.0, &permutation_matrix, &permutation_matrix);

    test_writeresult(ppc_source, ppc_dest, permutation_matrix);
    //results.push_back(???);
  }

  std::cout << std::endl << std::endl << "Tests passed:" << std::endl;
  //for (std::size_t i = 0; i < diff_with_human.size(); ++i)
  //  std::cout << std::left << std::setw(16) << names[i] << "  " << diff_with_human[i] << std::endl;

  return (0);
}
