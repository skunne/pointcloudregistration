#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range
#include "cpr_graphmatching.h"

GraphMatching::GraphMatching(MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj)
  : vsim(vsim), esim(esim), g_adj(g_adj), h_adj(h_adj), matching(g_adj->rows(), h_adj->rows())
{
  // assert g_adj and h_adj are square matrices
  assert(g_adj->rows() == g_adj->cols());
  assert(h_adj->rows() == h_adj->cols());
}

void GraphMatching::run(void)
{

}
// void GraphMatching::run()
// {
//
//   // Initialisation
//   double lambda;
//   double lambda_new;
//   MatrixDouble p(g_adj->rows(), h_adj->cols());
//   MatrixDouble p_new(g_adj->rows(), h_adj->cols());
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

// double GraphMatching::f_smooth(MatrixDouble const *p) const
// {
//   double res = 0;       // res = - sum_{edge e in G} esim(e, matched(e)) / (nG * nH)
//   unsigned int eg = 0;  // index of edge in G
//   for (auto edge_g_itr = esim->sourceEdgeIndex.cbegin(); edge_g_itr != esim->sourceEdgeIndex.cend(); ++edge_g_itr)
//   {
//     assert(eg == esim->sourceEdgeIndex.at(edge_g_itr->first));
//     unsigned int mapped_vertex_1, mapped_vertex_2;
//     for (
//       mapped_vertex_1 = 0;
//       mapped_vertex_1 < p->cols() && (*p)(edge_g_itr->first.first, mapped_vertex_1) == 0;
//       ++mapped_vertex_1)
//       ;
//     for (
//       mapped_vertex_2 = 0;
//       mapped_vertex_2 < p->cols() && (*p)(edge_g_itr->first.second, mapped_vertex_2) == 0;
//       ++mapped_vertex_2)
//       ;
//     auto edgeToIndex_h_itr = esim->destEdgeIndex.find(std::make_pair(mapped_vertex_1, mapped_vertex_2));
//     if (edgeToIndex_h_itr != esim->destEdgeIndex.end())  // if edge in G mapped to edge in H
//       res -= esim->m(eg, edgeToIndex_h_itr->second);
//     else          // if edge in G not mapped to an edge in H
//       res -= 1.0; // this is the worst possible penalty, since esim->m is normalized
//   }
//   res /= (g_adj->rows() * h_adj->rows());
//   return res;
// }
//
// double GraphMatching::f(double lambda, MatrixDouble const *p) const
// {
//   return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
// }
