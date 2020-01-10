#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range

// #include <CGAL/QP_models.h>     // quadratic programming
// #include <CGAL/QP_functions.h>
// #include <CGAL/MP_Float.h>
// typedef CGAL::MP_Float ET;

#include "cpr_graphmatching.h"
#include "cpr_graphmatching_path.h"

GraphMatchingPath::GraphMatchingPath(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
  : GraphMatching(vsim, esim, g_adj, h_adj)
{
  std::size_t ng = g_adj->rows();
  std::size_t nh = h_adj->rows();
  std::size_t x_len = ng * nh;    // nb variables
  std::size_t u_len = ng + nh;
  std::size_t y_len = ng + nh;    // nb constraints
  std::size_t v_len = x_len;
  z = (double *) malloc((x_len + u_len + y_len + v_len) * sizeof(*z));
}

void GraphMatchingPath::run()
{
  // Initialisation
  double lambda;
  double lambda_new;
  Eigen::MatrixXd p(g_adj->rows(), h_adj->cols());
  Eigen::MatrixXd p_new(g_adj->rows(), h_adj->cols());

  lambda = 0;
  frankWolfe(lambda, &p, &p);

  double dlambda = 0.1;  // TODO find correct value
  double epsilon = 0.1;  // TODO find correct value

  // Cycle over lambda
  while (lambda < 1.0)
  {
    lambda_new = lambda + dlambda;
    if (abs(f(lambda_new, &p) - f(lambda, &p)) < epsilon)
    {
      lambda = lambda_new;
    }
    else
    {
      frankWolfe(lambda, &p_new, &p);   // maybe remove p argument and do memcpy(p_new, p) before call to frankWolfe()
      lambda = lambda_new;
    }
  }
  // Output
  // copy p into matching?
}

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
  (void) lambda;
  (void) p;
  return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
}

void GraphMatchingPath::frankWolfe(double lambda, Eigen::MatrixXd *x_return, Eigen::MatrixXd const *x_0)
{

}
