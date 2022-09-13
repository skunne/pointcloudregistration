#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>
#include <fstream>  // print matching to csv files
#include <string>             // converting string to float

#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>    // pcl::PointXYZ
#include <pcl/sample_consensus/ransac.h>  // pcl::RandomSampleConsensus
#include <pcl/sample_consensus/sac_model_registration.h>  // pcl::SampleConsensusModelRegistration

#include "cpr_loadfiles.h"
#include "cpr_params.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_graphmatching_frankwolfe.h" // if using FrankWolfe quadratic programming algorithm
//#include "cpr_graphmatching_nonlin.h"     // if using Ipopt nonlinear solver

#include "cpr_debug_supervoxel.h"

//std::vector<int> &test_applyransac(ProcessedPointCloud &ppc_source, ProcessedPointCloud ppc_dest, MatrixInt const &matching)
void test_applyransac(
  pcl::PointCloud<pcl::PointXYZ>::Ptr const pc_data, pcl::PointCloud<pcl::PointXYZ>::Ptr const pc_model,
  double const threshold,
  std::vector<int> &inliers)
{
  // std::vector<std::tuple<double,double,double>> pc_data = ppc_source.exportCentroidPointCloud_as_vector();
  // std::vector<std::tuple<double,double,double>> pc_model = ppc_dest.exportCentroidPointCloud_as_vector();
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZ>);
  //cloud_model = ppc_source.supervoxel_clusters.centroids

  pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr
    model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(pc_data));
  model->setInputTarget(pc_model);

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
  ransac.setDistanceThreshold(threshold);
  ransac.computeModel();
  //std::vector<int> inliers;
  ransac.getInliers(inliers);

  //return inliers;
}

int test_getmatch(int i, MatrixInt const &permutation_matrix)
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


void test_getPermutationIndices(MatrixInt const &permutation_matrix, std::vector<int> src_indices, std::vector<int> dst_indices)
{
  for (int i = 0; i < permutation_matrix.rows(); ++i)
  {
    int j = test_getmatch(i, permutation_matrix);
    if (j < permutation_matrix.cols())
    {
      src_indices.push_back(i);
      dst_indices.push_back(j);
    }
  }
}

// void test_printVoxels(ProcessedPointCloud const &ppc, std::string const &filename)
// {
//   pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = ppc.super.getLabeledCloud();
//   std::ofstream outfile;
//   outfile.open(filename);
//   for (auto point = cloud->begin (); point != cloud->end (); ++point)
//   {
//     outfile << *point << std::endl;
//   }
//   outfile.close();
// }

void test_writeresult(
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_source, pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dest,
  std::vector<int> const &inliers,
  std::string const &filenamesrc_allclusters, std::string const &filenamedst_allclusters,
  std::string const &filenamesrc_inliers, std::string const &filenamedst_inliers
)
{
  /* OUTPUT ALL CLUSTERS BEFORE RANSAC */

  std::fstream src_out_allclusters(filenamesrc_allclusters, std::fstream::out | std::fstream::trunc);
  std::fstream dst_out_allclusters(filenamedst_allclusters, std::fstream::out | std::fstream::trunc);
  src_out_allclusters << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  dst_out_allclusters << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  if (!src_out_allclusters)
    errorLoadingFile("output", filenamesrc_allclusters.c_str());
  if (!dst_out_allclusters)
    errorLoadingFile("output", filenamedst_allclusters.c_str());
  for (unsigned int i = 0; i < pc_source->width; ++i)
  {
    src_out_allclusters << i << ',' << pc_source->at(i) << std::endl;
    dst_out_allclusters << i << ',' << pc_dest->at(i) << std::endl;
  }

  /* OUTPUT INLIER CLUSTERS AFTER RANSAC */

  std::fstream src_out_inliers(filenamesrc_inliers, std::fstream::out | std::fstream::trunc);
  std::fstream dst_out_inliers(filenamedst_inliers, std::fstream::out | std::fstream::trunc);
  src_out_inliers << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  dst_out_inliers << "id,dimension_1,dimension_2,dimension_3" << std::endl;
  if (!src_out_inliers)
    errorLoadingFile("output", filenamesrc_inliers.c_str());
  if (!dst_out_inliers)
    errorLoadingFile("output", filenamedst_inliers.c_str());
  for (int i: inliers)
  {
    src_out_inliers << i << ',' << pc_source->at(i) << std::endl;
    dst_out_inliers << i << ',' << pc_dest->at(i) << std::endl;
  }
  // assert(clusters_dest.size() == permutation_matrix.cols());
  //
  // test_printVoxels(ppc_source, filenamesrc);
  // test_printVoxels(ppc_dest, filenamedst);
}

int test_printUsage(char const *cmd)
{
  std::cout << "SYNOPSIS" << std::endl << std::endl;
  std::cout << cmd << " <meta> <pc src> <pc dst> <out>" << std::endl;
  std::cout << "    Run supervoxel clustering and graph-matching on two pointclouds." << std::endl;
  std::cout << "OUTPUT" << std::endl;
  std::cout << "    Results are printed to two text files src_<out> and dst_<out>." << std::endl;
  std::cout << "    One point per line, with labels." << std::endl;
  std::cout << "    Two points in same file have same label iff they belong to same supervoxel." << std::endl;
  std::cout << "    Two labels in different files are equal iff those clusters are matched by graph-matching." << std::endl;
  std::cout << "EXAMPLE USE" << std::endl;
  std::cout << cmd << " heart.meta heart.pcd heart_rotated.pcd out.txt" << std::endl;
  return 1;
}

// void test_printVoxels(ProcessedPointCloud const &ppc, std::string const &filename)
// {
//   pcl::PointCloud<pcl::PointXYZL>::Ptr cloud = ppc.super.getLabeledCloud();
//   std::ofstream outfile;
//   outfile.open(filename);
//   for (auto point = cloud->begin (); point != cloud->end (); ++point)
//   {
//     outfile << *point << std::endl;
//   }
//   outfile.close();
// }

int main(int argc, char ** argv)
{
  /* READ ARGUMENTS */

  if (argc < 5)
    return test_printUsage(argv[0]);

  //std::string out_folder = "/SCRATCH-BIRD/users/skunne/";
  std::string out_filename(argv[4]);

  Params params(argv[1]);
  params.filename = argv[2];
  ProcessedPointCloud ppc_source(params);
  params.filename = argv[3];
  ProcessedPointCloud ppc_dest(params);

  if (ppc_source.error() || ppc_dest.error())
    return 1;

  //test_printVoxels(ppc_source, "src_" + out_filename);
  //test_printVoxels(ppc_dest, "dst_" + out_filename);

  /* SUPERVOXEL CLUSTERING AND FEATURES */

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  build_error = ppc_dest.build();
  if (build_error)
    return build_error;

  /* GRAPH MATCHING */

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  printMatrixToFile("esf_similarity_matrix", vsim_mat.m);
  //printMatrixToFile("edge_similarity_matrix", esim_mat.m);

  GraphMatchingFrankwolfe gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  // gm.frankWolfe(0.0, &permutation_matrix, &permutation_matrix);
  // GraphMatchingNonlin gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);
  gm.run();

  /* APPLY PERMUTATION TO CLUSTER CENTROIDS */

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_src(ppc_source.exportCentroidPointCloud_as_pcl());
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_dst(ppc_dest.exportCentroidPointCloud_as_pcl());

  std::vector<int> src_indices;
  std::vector<int> dst_indices;
  test_getPermutationIndices(gm.matching, src_indices, dst_indices);

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_data(new pcl::PointCloud<pcl::PointXYZ>(*pc_src, src_indices));
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_model(new pcl::PointCloud<pcl::PointXYZ>(*pc_dst, dst_indices));

  /* RANSAC */

  double const threshold = 2 * ppc_source.get_seed_resolution();
  std::vector<int> inliers;
  test_applyransac(pc_data, pc_model, threshold, inliers);

  /* OUTPUT CENTROIDS WITH SRC,DST IN SELF-CONSISTENT PERMUTATION ORDER */
  test_writeresult(pc_data, pc_model,
                   inliers,
                   "src_allclusters_" + out_filename, "dst_allclusters_" + out_filename,
                   "src_inliers_" + out_filename, "dst_inliers_" + out_filename);

  /* OUTPUT PERMUTATION MATRIX */
  std::fstream matrix_out("mat.txt", std::fstream::out | std::fstream::trunc);
  if (!matrix_out)
    errorLoadingFile("output", "mat.txt");
  matrix_out << gm.matching << std::endl;

  return (0);
}
