
#include <Eigen/Core>

#include "test_frankwolfe.h"
#include "cpr_debug_visualisation.h"

#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_visualisation.h"
#include "cpr_graphmatching_path.h"

KeyT testwpc_find_one_matched_point(KeyT point_g, MatrixDouble const &m)
{
  //pcl::console::print_error("point_h goes from 0...");
  KeyT point_h = 0;
  for (
    point_h = 0;
    point_h < m.cols() &&
        abs(1.0 - m(point_g, point_h)) >= 0.01;
    ++point_h
  );
  //pcl::console::print_error("to %u\n", point_h);
  return point_h;
}

// populate samplepoints_h such that
// matching(samplepoints_g[i], samplepoints_h) == 1.0
int testwpc_find_matching_points(std::vector<KeyT> const &samplepoints_g, std::vector<KeyT> &samplepoints_h, MatrixDouble const &matching)
{
  int nb_points_without_matching = 0;
  samplepoints_h.resize(samplepoints_g.size());
  for (std::size_t i = 0; i < samplepoints_g.size(); ++i)
  {
    //pcl::console::print_error("WE'RE IN A LOOP   i = %d\n", i);
    samplepoints_h[i] = testwpc_find_one_matched_point(samplepoints_g[i], matching);
    if (samplepoints_h[i] >= matching.cols())
      ++nb_points_without_matching;
  }
  cprdbg::visualisation::print_both_sets_of_sample_points(samplepoints_g, samplepoints_h);
  return nb_points_without_matching;
}

double test_with_pointclouds (int argc, char const *const *argv)
{
  //////////////
  // Load files, run supervoxel clustering, add features
  //////////////

  if (argc < 3)
    return printUsage(argv[0]);

  ProcessedPointCloud ppc_source(argv[1]);
  ProcessedPointCloud ppc_dest(argv[2]);

  if (ppc_source.error() || ppc_dest.error())
    return 1;

  int build_error = ppc_source.build();
  if (build_error)
    return build_error;

  build_error = ppc_dest.build();
  if (build_error)
    return build_error;

  //////////////
  // Build similarity matrices from features
  //////////////

  VertexSimilarityMatrix vsim_mat(ppc_source.esf_descriptors, ppc_dest.esf_descriptors);
  EdgeSimilarityMatrix esim_mat(ppc_source.edge_descriptors, ppc_dest.edge_descriptors);

  //////////////
  // Perform graph matching
  //////////////

  GraphMatchingPath gm(&vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix);

  int const ng = ppc_source.getNbVertices();
  int const nh = ppc_dest.getNbVertices();

  MatrixDouble matching(ng, nh);
  matching.setZero();

  // run graph matching algorithm and compare resulting matching with identity matching
  double result =
    run_print_compare(ng, nh, &vsim_mat.m, &esim_mat, &ppc_source.adjacency_matrix, &ppc_dest.adjacency_matrix, &matching);//, &almost_identity);

  //////////////
  // Visualisation
  //////////////

  // find 8 arbitrary points in source and their image in dest
  std::vector<KeyT> samplepoints_source = {0, 10, 20, 30, 40, 50, 60, 70};
  std::vector<KeyT> samplepoints_dest;
  int nb_points_without_matching =
    testwpc_find_matching_points(samplepoints_source, samplepoints_dest, matching);
  if (nb_points_without_matching > 0)
    pcl::console::print_error("    %d points did not have a match!\n", nb_points_without_matching);

  // initialise visualisation object
  pcl::visualization::PCLVisualizer::Ptr viewer_source =
    ppc_source.visualise();
  pcl::visualization::PCLVisualizer::Ptr viewer_dest =
    ppc_dest.visualise();

  // add colours to the 8 arbitrary points
  ppc_source.addSomeColours(viewer_source, samplepoints_source);
  ppc_dest.addSomeColours(viewer_dest, samplepoints_dest);

  // display
  while (!viewer_source->wasStopped())
    viewer_source->spinOnce(100);
  while (!viewer_dest->wasStopped())
    viewer_dest->spinOnce(100);

  return result;
}
