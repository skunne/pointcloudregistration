#include <iostream>

#include "cpr_main.h"               // EdgeSimilarityMatrix, MatrixDouble, MatrixInt
#include "cpr_graphmatching_path.h" // GraphMatchingPath
//#include "cpr_matrices.h"           // EdgeSimilarityMatrix

double run_print_compare(int ng, int nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj, MatrixDouble const *humansolution)
{
  // declare graph matching algorithm
  GraphMatchingPath gm(vsim, esim, g_adj, h_adj);

  // declare stochastic matrix which will hold the solution graph matching
  MatrixDouble x(ng,nh);

  // initialise with trivial feasible solution
  x.fill(1.0 / (ng < nh ? nh : ng));

  // solve
  gm.frankWolfe(0.0, &x, &x);

  // compute scores
  double solved_score = gm.bilinear(x.data(), x.data());
  double human_score = gm.bilinear(humansolution->data(), humansolution->data());

  // output initial solution
  std::cout << "Starting solution:" << std::endl;
  std::cout << x << std::endl;

  // output final solution
  std::cout << std::endl << "Final solution:" << std::endl;
  std::cout << x << std::endl;

  // output final score and compare with human score
  std::cout << "Final similarity score:" << std::endl;
  std::cout << "    " << solved_score << std::endl;
  std::cout << std::endl << "Compare with human-known matching and score" << std::endl;

  std::cout << "Human-known matching:" << std::endl;
  std::cout << *humansolution << std::endl;
  std::cout << "Human-known score:" << std::endl;
  std::cout << "    " << human_score << std::endl;

  std::cout << std::endl << "Algorithm did ";
  if (solved_score < human_score)
    std::cout << "worse than human" << std::endl;
  if (solved_score > human_score)
    std::cout << "better than human" << std::endl;
  if (solved_score == human_score)
    std::cout << "as good as human" << std::endl;

  return (solved_score - human_score);
}
