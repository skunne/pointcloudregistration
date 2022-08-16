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

std::size_t test_getmatch(std::size_t i, MatrixInt const &permutation_matrix)
{
  int j;
  for (
    j = 0;
    j < permutation_matrix.cols() && permutation_matrix(i,j) < 0.00001;
    ++j
  );
  assert(j == permutation_matrix.cols() || abs(1-permutation_matrix(i,j)) < 0.00001);
  return (j);
}

void test_writeresult(ProcessedPointCloud const &ppc_source, ProcessedPointCloud const &ppc_dest, MatrixInt const &permutation_matrix, std::string const &filenamesrc, std::string const &filenamedst)
{
  SupervoxelClusters const &clusters_source = ppc_source.supervoxel_clusters;
  SupervoxelClusters const &clusters_dest = ppc_dest.supervoxel_clusters;

  std::fstream src_out(filenamesrc, std::fstream::out | std::fstream::trunc);
  std::fstream dst_out(filenamedst, std::fstream::out | std::fstream::trunc);

  src_out << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  dst_out << "id,dimension_1,dimension_2,dimension_3" << std::endl;

  if (!src_out)
    errorLoadingFile("output", filenamesrc.c_str());
  if (!dst_out)
    errorLoadingFile("output", filenamedst.c_str());

  assert(clusters_dest.size() == permutation_matrix.cols());

  for (std::size_t i = 0; i < clusters_source.size(); ++i)
  {
    std::size_t j = test_getmatch(i, permutation_matrix);
    if (j < clusters_dest.size())
    {
      for (auto const &p : *(clusters_source.at(i)->voxels_))
      {
        src_out << i << ',' << p.x << ',' << p.y << ',' << p.z << std::endl;
      }

      for (auto const &p : *(clusters_dest.at(j)->voxels_))
      {
        src_out << i << ',' << p.x << ',' << p.y << ',' << p.z << std::endl;
      }
      // src_out << i << ',' << std::get<0>(pc_source[i])
      //              << ',' << std::get<1>(pc_source[i])
      //              << ',' << std::get<2>(pc_source[i]) << std::endl;
      // dst_out << i << ',' << std::get<0>(pc_dest[j])
      //              << ',' << std::get<1>(pc_dest[j])
      //              << ',' << std::get<2>(pc_dest[j]) << std::endl;
    }
  }
}


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
  ProcessedPointCloud ppc_dest(argv[2]);

  if (ppc_source.error() || ppc_dest.error())
    return 1;


  //test_printVoxels(ppc_source, "src_" + out_filename);
  //test_printVoxels(ppc_dest, "dst_" + out_filename);

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  build_error = ppc_dest.build();
  if (build_error)
    return build_error;

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  printMatrixToFile("output/esf_similarity_matrix", vsim_mat.m);
  printMatrixToFile("output/edge_similarity_matrix", esim_mat.m);

  GraphMatchingFrankwolfe gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  // gm.frankWolfe(0.0, &permutation_matrix, &permutation_matrix);
  // GraphMatchingNonlin gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  gm.run();

  test_writeresult(ppc_source, ppc_dest, gm.matching, "src_" + out_filename, "src_" + out_filename);


  return (0);
}
