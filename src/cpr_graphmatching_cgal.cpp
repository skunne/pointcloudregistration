#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range

#include <CGAL/QP_models.h>     // quadratic programming
#include <CGAL/QP_functions.h>
#include <CGAL/MP_Float.h>
typedef CGAL::MP_Float ET;

#include "cpr_graphmatching.h"
#include "cpr_graphmatching_cgal.h"

GraphMatchingCgal::GraphMatchingCgal(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
: GraphMatching(vsim, esim, g_adj, h_adj)
{
}

void GraphMatchingCgal::run()
{
  std::stringstream in;
  fillMpsStream(in);
  pcl::console::print_info("Building problem object.\n");
  CGAL::Quadratic_program_from_mps<int> qp(in);
  assert (qp.is_valid()); // we should have a valid mps file
  // solve the program, using ET as the exact type
  pcl::console::print_info("Solving problem.\n");
  CGAL::Quadratic_program_solution<ET> s = CGAL::solve_quadratic_program(qp, ET());
  // output solution
  pcl::console::print_info("Printing solution.\n");
  std::cout << s;
}

void GraphMatchingCgal::fillMpsStream(std::stringstream &in) const
{
  pcl::console::print_info("Writing qp problem as mps stringstream.\n");
  in << "NAME qp" << std::endl
     << "ROWS" << std::endl
     << "N obj" << std::endl;
  for (int lin = 0; lin < g_adj->rows(); ++lin)
    in << "E stochastic_lin" << lin << std::endl; // sol matrix is line-stochastic
  for (int col = 0; col < h_adj->rows(); ++col)
    in << "E stochastic_col" << col << std::endl; // sol matrix is column-stochastic
  in << "COLUMNS" << std::endl;
  for (int lin = 0; lin < g_adj->rows(); ++lin)   // sol matrix is line-stochastic
    for (int col = 0; col < h_adj->rows(); ++col)
      in << 'x' << lin << col << " stochastic_lin" << lin << 1 << std::endl;
  for (int col = 0; col < h_adj->rows(); ++col)   // sol matrix is column-stochastic
    for (int lin = 0; lin < g_adj->rows(); ++lin)
      in << 'x' << lin << col << " stochastic_col" << col << 1 << std::endl;
  //for (int ig = 0; ig < g_adj->rows(); ++ig)      // linear term of objective function
  //  for (int ih = 0; ih < h_adj->rows(); ++ih)
  //    in << 'x' << ig << ih << " obj" << (*vsim)(ig, ih) << std::endl;
  in << "RHS" << std::endl;
  for (int lin = 0; lin < g_adj->rows(); ++lin)
    in << "rhs stochastic_lin" << lin << 1 << std::endl; // sol matrix is line-stochastic
  for (int col = 0; col < h_adj->rows(); ++col)
    in << "rhs stochastic_col" << col << 1 << std::endl; // sol matrix is column-stochastic
  pcl::console::print_info("Writing dmatrix to mps stringstream.\n");
  //inefficient way of writing quadratic objective
  in << "DMATRIX" << std::endl;
  for (int ig = 0; ig < g_adj->rows(); ++ig)
    for (int ih = 0; ih < h_adj->rows(); ++ih)
      for (int jg = 0; jg < g_adj->rows(); ++jg)
        for (int jh = 0; jh < h_adj->rows(); ++jh)
        {
          if (ig == jg && ih == jh)       // node similarity
            in << 'x' << ig << jg << " x" << ih << jh << ' ' << (*vsim)(ig, ih) << std::endl;
          else if (ig != jg && ih != jh)  // edge similarity
          {
            auto eg = esim->sourceEdgeIndex.find(std::make_pair(ig, jg));
            auto eh = esim->destEdgeIndex.find(std::make_pair(ih, jh));
            double edge_sim;
            if (eh == esim->destEdgeIndex.end() || eg == esim->sourceEdgeIndex.end())
              edge_sim = 0;     // ig,jg or ih,jh is not an edge
            else
            {
              edge_sim = esim->m(eg->second, eh->second);
            }
            in << 'x' << ig << jg << " x" << ih << jh << ' ' << edge_sim << std::endl;
          }
          // else: 0
        }
  // //slightly less inefficient way of writing objective
  // in << "QUADOBJ" << std::endl;
  // for (int )

  pcl::console::print_info("Finished writing problem!\n");
}
// void GraphMatchingCgal::run()
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

// double GraphMatchingCgal::f_smooth(Eigen::MatrixXd const *p) const
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
// double GraphMatchingCgal::f(double lambda, Eigen::MatrixXd const *p) const
// {
//   return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
// }
