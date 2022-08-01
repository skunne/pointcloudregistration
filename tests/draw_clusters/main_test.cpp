#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <fstream>
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
  std::cout << cmd << " <meta> <pc> <out>" << std::endl;
  std::cout << "    Apply supervoxel to pointcloud <pc> with params from .meta file <meta>." << std::endl;
  std::cout << "    <meta> must be the name of a metadata file. The point cloud filename given inside <meta> is ignored; only the parameters are used." << std::endl;
  std::cout << "OUTPUT" << std::endl;
  std::cout << "    Results are printed to text file <out>:." << std::endl;
  std::cout << "    one point per line, with labels. Two points have same label iff they belong to same supervoxel." << std::endl;
  std::cout << "EXAMPLE USE" << std::endl;
  std::cout << cmd << " heart.meta heart.pcd" << std::endl;
  return 1;
}

void test_printVoxels(ProcessedPointCloud const &ppc, std::string const &filename)
{
  pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = ppc.super.getLabeledCloud();
  std::ofstream outfile;
  outfile.open(filename);
  for (auto point = cloud->begin (); point != cloud->end (); ++point)
  {
    outfile << *point << std::endl;
  }
  outfile.close();
}

int main(int argc, char ** argv)
{
  if (argc < 4)
    return test_printUsage(argv[0]);

  std::string out_folder = "/SCRATCH-BIRD/users/skunne/";
  std::string out_filename(argv[3]);

  Params params(argv[1]);
  params.filename = argv[2];
  ProcessedPointCloud ppc_source(params);
  //ProcessedPointCloud ppc_dest(argv[2]);

  if (ppc_source.error())// || ppc_dest.error())
    return 1;

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  test_printVoxels(ppc_source, out_filename);

  // build_error = ppc_dest.build();
  // if (build_error)
  //   return build_error;

  // VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  // EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);
  //
  // printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  // printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

  //GraphMatchingFrankwolfe gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  //gm.frankWolfe(0.0, &permutation_matrix, &permutation_matrix);
  //GraphMatchingNonlin gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  //gm.run();

  return (0);
}
