#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range

#include <CGAL/QP_models.h>     // quadratic programming
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>

#include "cpr_graphmatching.h"
#include "cpr_graphmatching_path.h"

GraphMatchingPath::GraphMatchingPath(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
  : GraphMatching(vsim, esim, g_adj, h_adj)
{
}

void GraphMatchingPath::run(void)
{
  frankWolfe(0.0, NULL, NULL);
}
// void GraphMatchingPath::run()
// {
//
//   // Initialisation
//   double lambda;
//   double lambda_new;
//   Eigen::MatrixXd p(g_adj->rows(), h_adj->cols());
//   Eigen::MatrixXd p_new(g_adj->rows(), h_adj->cols());
//
//   lambda = 0;
//   frankWolfe(lambda, &p, &p);
//
//   double dlambda = 0.01;   // this is completely false, please change this
//   double epsilon = 0.1;    // this is completely false, please change this
//
//   // Cycle over lambda
//   while (lambda < 1.0)
//   {
//     lambda_new = lambda + dlambda;
//     if (abs(f(lambda_new, &p) - f(lambda, &p)) < epsilon)
//     {
//       lambda = lambda_new;
//     }
//     else
//     {
//       frankWolfe(lambda, &p_new, &p);   // maybe remove p argument and do memcpy(p_new, p) before call to frankWolfe()
//       lambda = lambda_new;
//     }
//   }
//
//   // Output
//   // copy p into matching?
// }

double GraphMatchingPath::f_smooth(Eigen::MatrixXd const *p) const
{
  double res = 0;       // res = - sum_{edge e in G} esim(e, matched(e)) / (nG * nH)
  unsigned int eg = 0;  // index of edge in G
  for (auto edge_g_itr = esim->sourceEdgeIndex.cbegin(); edge_g_itr != esim->sourceEdgeIndex.cend(); ++edge_g_itr)
  {
    assert(eg == esim->sourceEdgeIndex.at(edge_g_itr->first));
    unsigned int mapped_vertex_1, mapped_vertex_2;
    for (
      mapped_vertex_1 = 0;
      mapped_vertex_1 < p->cols() && (*p)(edge_g_itr->first.first, mapped_vertex_1) == 0;
      ++mapped_vertex_1)
      ;
    for (
      mapped_vertex_2 = 0;
      mapped_vertex_2 < p->cols() && (*p)(edge_g_itr->first.second, mapped_vertex_2) == 0;
      ++mapped_vertex_2)
      ;
    auto edgeToIndex_h_itr = esim->destEdgeIndex.find(std::make_pair(mapped_vertex_1, mapped_vertex_2));
    if (edgeToIndex_h_itr != esim->destEdgeIndex.end())  // if edge in G mapped to edge in H
      res -= esim->m(eg, edgeToIndex_h_itr->second);
    else          // if edge in G not mapped to an edge in H
      res -= 1.0; // this is the worst possible penalty, since esim->m is normalized
  }
  res /= (g_adj->rows() * h_adj->rows());
  return res;
}

double GraphMatchingPath::f(double lambda, Eigen::MatrixXd const *p) const
{
  return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
}

void GraphMatchingPath::frankWolfe(double lambda, Eigen::MatrixXd *p_new, Eigen::MatrixXd const *p_start)
{
  // by default, we have a nonnegative QP with Ax <= b
  CGAL::Quadratic_program<int> qp (CGAL::SMALLER, true, 0, false, 0);

  // now set the non-default entries:
  const int X = 0;
  const int Y = 1;
  qp.set_a(X, 0,  1); qp.set_a(Y, 0, 1); qp.set_b(0, 7);  //  x + y  <= 7
  qp.set_a(X, 1, -1); qp.set_a(Y, 1, 2); qp.set_b(1, 4);  // -x + 2y <= 4
  qp.set_u(Y, true, 4);                                   //       y <= 4
  qp.set_d(X, X, 2); qp.set_d (Y, Y, 8); // !!specify 2D!!    x^2 + 4 y^2
  qp.set_c(Y, -32);                                       // -32y
  qp.set_c0(64);                                          // +64
  CGAL::Quadratic_program_solution<CGAL::MP_Float> s = CGAL::solve_quadratic_program(qp, CGAL::MP_Float());
  assert (s.solves_quadratic_program(qp));
  // output solution
  std::cout << s;
}
