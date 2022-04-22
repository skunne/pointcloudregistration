#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>
#include <fstream>  // print matching to csv files
#include <string>             // converting string to float

#include "cpr_loadfiles.h"
#include "cpr_params.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_graphmatching_frankwolfe.h" // if using FrankWolfe quadratic programming algorithm
//#include "cpr_graphmatching_nonlin.h"     // if using Ipopt nonlinear solver

#include "cpr_debug_supervoxel.h"


int test_printUsage(char const *cmd)
{
  std::cout << "SYNOPSIS" << std::endl << std::endl;
  std::cout << cmd << " <meta> <pc0> <v0> <s0> <pc1> <v1> <s1> [<pc2> <v2> <s2> ... <pcn> <vn> <sn>]" << std::endl;
  std::cout << "    Use the algorithm to register <pc 2k> against <pc 2k+1>" << std::endl;
  std::cout << "    <meta> must be the name of a metadata file. The point cloud filename given inside <meta> is ignored; only the parameters are used." << std::endl;
  std::cout << "    For each point cloud <pci>, specify the voxel resolution <vi> and seed resolution <si>" << std::endl;
  std::cout << "EXAMPLE USE" << std::endl;
  std::cout << cmd << " heart.meta heart.pcd 30 150 heart_rotated.pcd 30 150" << std::endl;
  return 1;
}

/*
std::size_t test_getmatch(std::size_t i, MatrixInt const &permutation_matrix)
{
  int j;
  for (
    j = 0;
    j < permutation_matrix.cols() && permutation_matrix(i,j) == 0;
    ++j
  );
  assert(j == permutation_matrix.cols() || permutation_matrix(i,j) == 1);
  return (j);
}
*/
std::size_t test_getmatch(std::size_t i, MatrixInt const &permutation_matrix)
{
  return i;
}

void test_writeresult(ProcessedPointCloud const &ppc_source, ProcessedPointCloud const &ppc_dest, MatrixInt const &permutation_matrix, char const *filenamesrc, char const *filenamedst)
{
  std::vector<std::tuple<double,double,double>> pc_source = ppc_source.exportPointCloud();
  std::vector<std::tuple<double,double,double>> pc_dest = ppc_dest.exportPointCloud();

  std::cout << "Source pointcloud:" << std::endl;
  for (auto &p : pc_source)
    std::cout << std::get<0>(p) << ',' << std::get<1>(p) << ',' << std::get<2>(p) << ',' << std::endl;

  std::cout << "Dest pointcloud:" << std::endl;
  for (auto &p : pc_dest)
    std::cout << std::get<0>(p) << ',' << std::get<1>(p) << ',' << std::get<2>(p) << ',' << std::endl;

  std::fstream src_out(filenamesrc, std::fstream::out | std::fstream::trunc);
  std::fstream dst_out(filenamedst, std::fstream::out | std::fstream::trunc);

  src_out << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  dst_out << "id,dimension_1,dimension_2,dimension_3" << std::endl;

  if (!src_out)
    errorLoadingFile("output", filenamesrc);
  if (!dst_out)
    errorLoadingFile("output", filenamedst);

  assert(pc_dest.size() == permutation_matrix.cols());
  // std::cout << "Permutation matrix:" << std::endl;
  // std::cout << permutation_matrix << std::endl;

  for (std::size_t i = 0; i < pc_source.size(); ++i)
  {
    std::size_t j = test_getmatch(i, permutation_matrix);
    if (j < pc_dest.size())
    {
      src_out << i << ',' << std::get<0>(pc_source[i])
                   << ',' << std::get<1>(pc_source[i])
                   << ',' << std::get<2>(pc_source[i]) << std::endl;
      dst_out << i << ',' << std::get<0>(pc_dest[j])
                   << ',' << std::get<1>(pc_dest[j])
                   << ',' << std::get<2>(pc_dest[j]) << std::endl;
    }
  }
}

int main(int argc, char ** argv)
{
  if (argc <= 7 || argc % 6 != 2)
    return test_printUsage(argv[0]);

  std::string out_folder = "/SCRATCH-BIRD/users/skunne/";

//  std::vector<char const *> names;
//  std::vector<double> results;

  Params params_source(argv[1]);

  Params params_dest = params_source;
  for (int i = 2; i < argc; i += 6)
  {
    /* SOURCE */
    params_source.filename = argv[i];
    params_source.voxel_resolution = std::stof(argv[i+1]);
    params_source.seed_resolution = std::stof(argv[i+2]);
    assert(params_source.error == 0);

    ProcessedPointCloud ppc_source(params_source);
    assert(ppc_source.error() == 0);
    int source_build_error = ppc_source.build();

    //std::cerr << "main: print pointcloud source" << std::endl;
    //cprdbg::supervoxel::print_pointcloud(ppc_source.cloud, 2);

    assert(source_build_error == 0);

    /* DESTINATION */
    params_dest.adjacency_filename = "/SCRATCH-BIRD/users/skunne/dest.adj";
    params_dest.filename = argv[i+3];
    params_dest.voxel_resolution = std::stof(argv[i+4]);
    params_dest.seed_resolution = std::stof(argv[i+5]);
    ProcessedPointCloud ppc_dest(params_dest);
    assert(ppc_dest.error() == 0);
    int dest_build_error = ppc_dest.build();
    assert(dest_build_error == 0);

    VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
    EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

    // std::cout << "VertexSimilarityMatrix: " << std::endl;
    // for (i = 0; i < 3; i++)
    // {
    //   std::cout << vsim_mat.m(i,0) << ' ' << vsim_mat.m(i,1) << ' ' << vsim_mat.m(i,2) << std::endl;
    // }
    // std::cout << "EgeSimilarityMatrix: " << std::endl;
    // for (i = 0; i < 3; i++)
    // {
    //   std::cout << esim_mat.m(i,0) << ' ' << esim_mat.m(i,1) << ' ' << esim_mat.m(i,2) << std::endl;
    // }
    //int const n_source = ppc_source.getNbVertices();
    //int const n_dest = ppc_dest.getNbVertices();

    //MatrixDouble permutation_matrix(n_source, n_dest);
    //permutation_matrix.fill(1.0 / (n_source < n_dest ? n_dest : n_source));
    GraphMatchingFrankwolfe gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
    //gm.frankWolfe(0.0, &permutation_matrix, &permutation_matrix);
    //GraphMatchingNonlin gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
    gm.run();

    std::cout << "Permutation matrix:" << std::endl;
    std::cout << gm.matching << std::endl;

    std::stringstream srcoutfilename;
    srcoutfilename << out_folder << "matched_" << argv[i] << "_src_" << 'v' << argv[i+1] << 's' << argv[i+2] << 'v' << argv[i+4] << 's' << argv[i+5] << ".csv";
    std::stringstream dstoutfilename;
    dstoutfilename << out_folder << "matched_" << argv[i+3] << "_dst_" << 'v' << argv[i+1] << 's' << argv[i+2] << 'v' << argv[i+4] << 's' << argv[i+5] << ".csv";
    std::cout << "srcoutfilename: " << srcoutfilename.str().c_str() << std::endl;
    test_writeresult(ppc_source, ppc_dest, gm.matching, srcoutfilename.str().c_str(), dstoutfilename.str().c_str());
    //results.push_back(???);
  }

  //std::cout << std::endl << std::endl << "Tests passed:" << std::endl;
  //for (std::size_t i = 0; i < diff_with_human.size(); ++i)
  //  std::cout << std::left << std::setw(16) << names[i] << "  " << diff_with_human[i] << std::endl;

  return (0);
}
