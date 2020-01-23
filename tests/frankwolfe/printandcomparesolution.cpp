#include <iostream>
#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

#include "cpr_main.h"               // EdgeSimilarityMatrix, MatrixDouble, MatrixInt
#include "cpr_matrices.h"           // printVectorAsMatrix<T>
#include "cpr_graphmatching_path.h" // GraphMatchingPath
//#include "cpr_matrices.h"           // EdgeSimilarityMatrix

double run_print_compare(std::size_t ng, std::size_t nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj, MatrixDouble const *humansolution)
{
  // declare graph matching algorithm
  GraphMatchingPath gm(vsim, esim, g_adj, h_adj);

  // declare stochastic matrix which will hold the solution graph matching
  MatrixDouble x(ng,nh);
  //std::vector<double> x;

  // initialise with trivial feasible solution
  x.fill(1.0 / (ng < nh ? nh : ng));
  //std::fill(x.begin(), x.end(), 1.0 / (ng < nh ? nh : ng));

  // output initial solution
  std::cout << "Starting solution:" << std::endl;
  std::cout << x << std::endl;
  //printVectorAsMatrix(x, ng, nh);

  // solve
  gm.frankWolfe(0.0, &x, &x);

  // compute scores
  std::vector<double>humancopyvector;
  humancopyvector.assign(humansolution->data(), humansolution->data()+ng*nh);
  std::vector<double>xcopy;
  xcopy.assign(x.data(), x.data()+ng*nh);
  double solved_score = gm.bilinear(xcopy, xcopy);
  double human_score = gm.bilinear(humancopyvector, humancopyvector);

  // output final solution
  std::cout << std::endl << "Final solution:" << std::endl;
  std::cout << x << std::endl;
  //printVectorAsMatrix(x, ng, nh);

  // output final score and compare with human score
  std::cout << "Final similarity score:" << std::endl;
  std::cout << "    " << solved_score << std::endl;
  std::cout << std::endl << "Compare with human-known matching and score" << std::endl;

  std::cout << "Human-known matching:" << std::endl;
  std::cout << *humansolution << std::endl;
  //printVectorAsMatrix(humancopyvector, ng, nh);
  std::cout << "Human-known score:" << std::endl;
  std::cout << "    " << human_score << std::endl;

  std::cout << std::endl << "Algorithm did ";
  if (abs(solved_score - human_score) < 0.00001)
    std::cout << "as good as human" << std::endl;
  else if (solved_score < human_score)
    std::cout << "worse than human" << std::endl;
  else if (solved_score > human_score)
    std::cout << "better than human" << std::endl;

  return (solved_score - human_score);
}

void print_similarity_matrices(MatrixDouble const &vsim, MatrixDouble const &esim)
{
  // output the chosen similarity matrices
  std::cout << std::fixed << std::setprecision(4);
  std::cout << "Vertex similarity matrix:" << std::endl;
  std::cout << vsim << std::endl << std::endl;
  std::cout.unsetf(std::ios_base::floatfield);
  std::cout << std::setprecision(1);
  std::cout << "Edge similarity matrix:" << std::endl;
  std::cout << esim << std::endl << std::endl;
  std::cout << std::fixed << std::setprecision(2);
}

void print_matrix_D(std::size_t ng, std::size_t nh, MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim)
{
  MatrixDouble D(ng*nh, ng*nh);
  D.setZero();
  for (auto const edge_g : esim->sourceEdgeIndex)
  {
    for (auto const edge_h : esim->sourceEdgeIndex)
    {
      std::size_t kx = edge_g.first.first * nh + edge_h.first.first;
      std::size_t ky = edge_g.first.second * nh + edge_h.first.second;
      D(kx,ky) = esim->m(edge_g.second, edge_h.second);
    }
  }
  for (KeyT ig = 0; ig < ng; ++ig)
    for (KeyT ih = 0; ih < nh; ++ih)
      D(ig * nh + ih, ig * nh + ih) = (*vsim)(ig, ih);

  std::cout << "Printing matrix D" << std::endl;
  std::cout << D << std::endl << std::endl;
}
