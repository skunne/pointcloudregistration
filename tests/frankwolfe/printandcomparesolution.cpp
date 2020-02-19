#include <iostream>
#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

#include "cpr_main.h"               // EdgeSimilarityMatrix, MatrixDouble, MatrixInt
#include "cpr_matrices.h"           // printVectorAsMatrix<T>
#include "cpr_graphmatching_frankwolfe.h" // GraphMatchingFrankwolfe
//#include "cpr_matrices.h"           // EdgeSimilarityMatrix

#include "test_frankwolfe.h"

void set_almost_identity(MatrixDouble &almost_identity, int ng, int nh);

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble const *humansolution, char const *human_solution_name,
  MatrixDouble *return_solution)
{
  // declare graph matching algorithm
  GraphMatchingFrankwolfe gm(vsim, esim, g_adj, h_adj);

  // declare stochastic matrix which will hold the solution graph matching
  MatrixDouble x(ng,nh);
  //std::vector<double> x;

  // initialise with trivial feasible solution
  x.fill(1.0 / (ng < nh ? nh : ng));
  //std::fill(x.begin(), x.end(), 1.0 / (ng < nh ? nh : ng));

  // output initial solution
  if (ng < 20 && nh < 20)
  {
    std::cout << "Starting solution:" << std::endl;
    std::cout << x << std::endl;
  }
  //printVectorAsMatrix(x, ng, nh);

  // solve
  gm.frankWolfe(0.0, &x, &x);

  // MatrixDouble almost_identity(ng, nh);
  // set_almost_identity(almost_identity, ng, nh);  // rectangular matrix, (Identity Block | Zero Block)

  // compute scores
  std::vector<double>humancopyvector;
  humancopyvector.assign(humansolution->data(), humansolution->data()+ng*nh);
  std::vector<double>xcopy;
  xcopy.assign(x.data(), x.data()+ng*nh);
  double solved_score = gm.bilinear(xcopy, xcopy);
  double human_score = gm.bilinear(humancopyvector, humancopyvector);

  // output final solution
  if (ng < 20 && nh < 20)
  {
    std::cout << std::endl << "Final solution:" << std::endl;
    std::cout << x << std::endl;
    //printVectorAsMatrix(x, ng, nh);
  }

  // output final score and compare with human score
  std::cout << "Final similarity score:" << std::endl;
  std::cout << "    " << solved_score << std::endl;
  std::cout << std::endl << "Compare with "<< human_solution_name << " matching score" << std::endl;

  if (ng < 20 && nh < 20)
  {
    std::cout << human_solution_name << " matching:" << std::endl;
    std::cout << *humansolution << std::endl;
    //printVectorAsMatrix(humancopyvector, ng, nh);
  }
  std::cout << human_solution_name << " score:" << std::endl;
  std::cout << "    " << human_score << std::endl;

  std::cout << std::endl << "Algorithm did ";
  if (abs(solved_score - human_score) < 0.00001)
    std::cout << "as good as " << human_solution_name << std::endl;
  else if (solved_score < human_score)
    std::cout << "worse than " << human_solution_name << std::endl;
  else if (solved_score > human_score)
    std::cout << "better than " << human_solution_name << std::endl;

  if (return_solution != NULL)
    *return_solution = x;

  return (solved_score - human_score);
}

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj)//, MatrixDouble const *humansolution)
{
  MatrixDouble almost_identity(ng, nh);
  set_almost_identity(almost_identity, ng, nh);  // rectangular matrix, (Identity Block | Zero Block)

  return run_print_compare(ng, nh, vsim, esim, g_adj, h_adj, &almost_identity, "Identity");
}

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble const *humansolution, char const *human_solution_name)
{
  return run_print_compare(ng, nh, vsim, esim, g_adj, h_adj, humansolution, human_solution_name, NULL);
}

double run_print_compare(std::size_t ng, std::size_t nh,
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj,
  MatrixDouble *return_solution)
{
  MatrixDouble almost_identity(ng, nh);
  set_almost_identity(almost_identity, ng, nh);  // rectangular matrix, (Identity Block | Zero Block)

  return run_print_compare(ng, nh, vsim, esim, g_adj, h_adj, &almost_identity, "Identity", return_solution);
}

void set_almost_identity(MatrixDouble &almost_identity, int ng, int nh)
{
  if (ng == nh)
  {
    almost_identity.setIdentity();
  }
  else if (ng < nh)
  {
    almost_identity.topLeftCorner(ng, ng).setIdentity();
    almost_identity.topRightCorner(ng, nh-ng).setZero();
  }
  else // nh < ng
  {
    almost_identity.topLeftCorner(nh, nh).setIdentity();
    almost_identity.bottomLeftCorner(ng-nh, nh).setZero();
  }
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
