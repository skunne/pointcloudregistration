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

int main(int argc, char ** argv)
{
  if (argc <= 3)
    return test_printUsage(argv[0]);

  //test_metricisgood(argv[1]);

  std::vector<char const *> names;// = { "two nodes", "houses", "5 vs 6", "multiple optimal", "pointclouds" };
  std::vector<double> results;//(5);
  //
  // names.push_back("two nodes");
  // diff_with_human.push_back(test_two_twonodes_graphs());

  // int ac = 3;
  // char const *av[3] = {"./tests/frankwolfe/test_frankwolfe", "metadata/big1.meta", "metadata/big3.meta"};
  // names.push_back("pointclouds");
  // diff_with_human.push_back(test_with_pointclouds(ac,av));
//  diff_with_human.push_back(test_with_pointclouds(argc, argv));

  Params params_source(argv[1]);
  params_source.filename = argv[2];
  assert(params_source.error == 0);

  ProcessedPointCloud ppc_source(params_source);
  assert(ppc_source.error() == 0);
  int source_build_error = ppc_source.build();
  assert(source_build_error == 0);

  Params params_dest = params_source;
  for (int i = 3; i < argc; ++i)
  {
    params_dest.filename = argv[i];
    ProcessedPointCloud ppc_dest(params_dest);
    assert(ppc_dest.error() == 0);
    int dest_build_error = ppc_dest.build();
    assert(dest_build_error == 0);

    VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
    EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

    GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);

    names.push_back(argv[i]);
    //results.push_back(???);
  }

  std::cout << std::endl << std::endl << "Tests passed:" << std::endl;
  //for (std::size_t i = 0; i < diff_with_human.size(); ++i)
  //  std::cout << std::left << std::setw(16) << names[i] << "  " << diff_with_human[i] << std::endl;

  return (0);
}
