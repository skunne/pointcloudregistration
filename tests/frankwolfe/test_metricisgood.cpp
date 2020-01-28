#include <fstream>    // print scores to file so they can be plotted by another script
#include <random>     // geenrate random transpositions

#include "cpr_processedpointcloud.h"
#include "cpr_graphmatching_path.h"
#include "cpr_loadfiles.h"  // print error message on open file

#include "test_frankwolfe.h"

/*
**  This test is to be used in conjunction with
**  the python script ../../scripts_python/display_scores.py
**
**  The function test_metricisgood() below will write a text file
**  output/scores_as_function_of_matching_quality.txt
**  consisting of several lines containing space-separated numbers
**  Each line if of the form:
**  c  score1 score2 score3 score4...
**  where c represents a number of transpositions (i,j)
**  and all the scores on the line are the result of the product x D x
**  where x is a permutation matrix obtained by multiplying c random transpositions
**  and D is the similarity matrix of the input pointcloud (given with the metadata_filename argument)
**
**  The python script display_scores.py will read the file output/scores_as_function_of_matching_quality.txt
**  And display a scatterplot (and save it to output/scores_as_function_of_matching_quality.png)
*/

void testmetric_printscorestofile(std::vector<std::vector<double>> const &scores)
{
  char const *filename = "output/scores_as_function_of_matching_quality.txt";
  std::fstream output(filename, std::fstream::out | std::fstream::trunc);

  pcl::console::print_info("    Saving scores to:\n      ");
  pcl::console::print_info("%s", filename);
  pcl::console::print_info("\n");
  if (!output)
    errorLoadingFile("output", filename);
  else
  {
    for (std::size_t complexity = 0; complexity < scores.size(); ++complexity)
    {
      output << complexity << ' ';
      for (double score : scores[complexity])
        output << score << ' ';
      output << std::endl;
    }
  }
}

void testmetric_swaprows(std::vector<double> &v, std::size_t width, std::size_t i, std::size_t j)
{
  double tmp;
  double *vi = &v[i * width];
  double *vj = &v[j * width];
  for (std::size_t col = 0; col < width; ++col)
  {
    tmp = vi[col];
    vi[col] = vj[col];
    vj[col] = tmp;
  }
}

// "x has complexity c" <==> x was constructed from c random transpositions
double testmetric_testonagraph(MatrixInt const &adj_m, VertexSimilarityMatrix const &vsim, EdgeSimilarityMatrix const &esim)
{
  std::cout << "testmetric_testonagraph()" << std::endl;

  std::size_t const width = adj_m.cols();
  std::size_t const nb_experiments = 1000;
  std::size_t const max_complexity = 3 * width;
  std::vector<std::vector<double>> scores(max_complexity);
  MatrixDouble identity_matrix(width, width);
  std::random_device randomd;
  std::vector<double> x; // graph-matching permutation matrix
  Eigen::RowVectorXd tmp_row(width);     // helper row vector for swapping rows in x

  GraphMatchingPath gm(&vsim.m, &esim, &adj_m, &adj_m);  // object that contains the metric

  identity_matrix.setIdentity();
  x.assign(identity_matrix.data(), identity_matrix.data() + width * width);

  scores[0].push_back(gm.bilinear(x, x));

  for (std::size_t k = 0; k < nb_experiments; ++k)
  {
    x.assign(identity_matrix.data(), identity_matrix.data() + width * width);
    //x.setIdentity();  // reset to complexity = 0
    for (std::size_t complexity = 1; complexity < max_complexity; ++complexity)
    {
      // random transposition (i, j)
      std::size_t i = randomd() % (width);
      std::size_t j = (randomd() % (width - 1));
      j = (j == i ? width - 1 : j);

      // compose x with transposition (i, j)
      testmetric_swaprows(x, width, i, j);

      // compute score
      scores[complexity].push_back(gm.bilinear(x, x));
    }
    std::cout << k << ' ' << std::flush;
  }

  testmetric_printscorestofile(scores);

  return 0.0;
}

double test_metricisgood(char const *metadata_filename)
{
  std::cout << "test_metricisgood()" << std::endl;
  //////////////
  // Load file
  //////////////

  ProcessedPointCloud ppc(metadata_filename);
  if (ppc.error())
    return 1;

  //////////////
  // Run supervoxel clustering and compute features
  //////////////

  int build_error = ppc.build();
  if (build_error)
    return 1;

  //////////////
  // Build similarity matrices against itself
  //////////////

  VertexSimilarityMatrix vsim(ppc.esf_descriptors, ppc.esf_descriptors);
  EdgeSimilarityMatrix esim(ppc.edge_descriptors, ppc.edge_descriptors);



  //////////////
  // Test the metric
  //////////////

  return testmetric_testonagraph(ppc.adjacency_matrix, vsim, esim);
}
