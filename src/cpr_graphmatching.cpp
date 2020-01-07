#include <stdexcept>    // std::out_of_range
#include "cpr_graphmatching.h"

GraphMatching::GraphMatching(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
  : vsim(vsim), esim(esim), g_adj(g_adj), h_adj(h_adj), matching(g_adj->rows(), h_adj->rows())
{
  // assert g_adj and h_adj are square matrices
}

void GraphMatching::run()
{

  // Initialisation
  double lambda;
  double lambda_new;
  Eigen::MatrixXd p(g_adj->rows(), h_adj->cols());
  Eigen::MatrixXd p_new(g_adj->rows(), h_adj->cols());

  lambda = 0;
  frankWolfe(lambda, &p, &p);

  double dlambda = 0.01;  // this is completely false, please change this
  double epsilon = 0.1;    // this is completely false, please change this

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
  // copy p into matching? but make sure this is p(1)?
}

double GraphMatching::f_smooth(Eigen::MatrixXd const *p) const
{
  // dres = - \sum_(vertex i in G) \sum_(neighbour j of i in G) | ||pi-pj|| - ||p(i')-p(j')|| | / (nG * nH)
  // where p(i) = point i, p(i') = point matched to i, nG,nH = number of nodes in G and H
  // ||p(i) - p(j)|| = distance in 3d space or in feature space??
  // dres = - sum_{edge e in G} esim(e, matched(e)) / (nG * nH)

  double dres = 0;

  unsigned int eg = 0;
  for (auto edge_g_itr = esim->sourceEdgeIndex.cbegin(); edge_g_itr != esim->sourceEdgeIndex.cend(); ++edge_g_itr)
  {
    // assert eg == esim.sourceEdgeIndex.at(edge_g_itr->first)
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
    if (mapped_vertex_1 < p->cols() && mapped_vertex_2 < p->cols()) // if both vertices map to vertices
    {
      try
      {
        unsigned int eh = esim->destEdgeIndex.at(std::make_pair(mapped_vertex_1, mapped_vertex_2));
        dres -= esim->m(eg, eh);
      }
      catch (const std::out_of_range &oor)  // if edge not mapped to an edge
      {
        dres -= 1.0; // this is the worst possible penalty, since esim is normalized
      }
    }
    else  // if one of vertices not mapped to a vertex
    {
      dres -= 1.0;  // this is the worst possible penalty, since esim is normalized
    }
  }
  dres /= (g_adj->rows() * h_adj->rows());
  return dres;
}
