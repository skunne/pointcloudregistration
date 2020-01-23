//#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range

// #include <CGAL/QP_models.h>     // quadratic programming
// #include <CGAL/QP_functions.h>
// #include <CGAL/MP_Float.h>
// typedef CGAL::MP_Float ET;

#include <cstdlib>  // malloc, free
#include <cstring>  // memcpy, memset
#include <algorithm> // std::fill
#include <vector>   // paranoia

#include "cpr_graphmatching.h"
#include "cpr_graphmatching_path.h"

#include "cpr_debug.h"

GraphMatchingPath::GraphMatchingPath(MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj)
  : GraphMatching(vsim, esim, g_adj, h_adj),
  ng(g_adj->rows()), nh(h_adj->rows()), x_len(ng * nh), nb_constraints(ng + nh),
  x(x_len), y(x_len), xD(x_len)
{
  // x.reserve(x_len);
  // y.reserve(x_len);
  // xD.reserve(x_len);

  cons_coeff_rowindex.push_back(1);   // duplicate coeff at pos 0 and pos 1 because apparently these three vectors should be 1-indexed??
  cons_coeff_colindex.push_back(1);
  cons_coeff_value.push_back(1.0);
  // stochasticity constraints: forall ig, sum_(ih) x(ig,ih) = 1
  for (std::size_t ig = 0; ig < ng; ++ig)
    for (std::size_t ih = 0; ih < nh; ++ih)
    {
      cons_coeff_rowindex.push_back(ig+1);    // 1-indexed int
      cons_coeff_colindex.push_back(ig * nh + ih + 1);
      cons_coeff_value.push_back(1.0);
    }
  // stochasticity constraints: forall ih, sum_(ig) x(ig,ih) = 1
  for (std::size_t ih = 0; ih < nh; ++ih)
    for (std::size_t ig = 0; ig < ng; ++ig)
    {
      cons_coeff_rowindex.push_back(ng+1+ih);    // continue from last row index
      cons_coeff_colindex.push_back(ig * nh + ih + 1);
      cons_coeff_value.push_back(1.0);
    }
  initSimplex(cons_coeff_rowindex, cons_coeff_colindex, cons_coeff_value);
}

GraphMatchingPath::~GraphMatchingPath()
{
}

void GraphMatchingPath::run()
{
  // Initialisation
  double lambda;
  double lambda_new;
  MatrixDouble p(ng, nh);
  MatrixDouble p_new(ng, nh);

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
  // when lambda == 1, all coeffs in p must be 0.0 or 1.0
  // copy p into this->matching and convert double to int
}

// input p = graph-matching matrix
// this one is probably the wrong version. see below
double GraphMatchingPath::f_smooth(MatrixDouble const *p) const
{
  // result = - sum_{edge e in G} esim(e, matched(e)) / (nG * nH)
    // article is unclear
    // this should maybe be abs(length(e) - length(matched(e)))
    // instead of esim(e, matched(e))
    // and anyway this only makes sense if p only has 0 and 1 coeffs
  double result = 0;

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
    auto edgeToIndex_h_itr = esim->destEdgeIndex.find(
      std::make_pair(mapped_vertex_1, mapped_vertex_2)
    );
    if (edgeToIndex_h_itr != esim->destEdgeIndex.end())  // if edge in G mapped to edge in H
      result += esim->m(eg, edgeToIndex_h_itr->second);
    else          // if edge in G not mapped to an edge in H
      result += 1.0 - 1.0; // this is the worst possible penalty, since esim->m is normalized
    ++eg;
  }
  result /= x_len;   // result = result / (ng * nh)
  return result;
}

// // this one is probably the correct version?
// double GraphMatchingPath::f_smooth(MatrixDouble const *p, EdgeDescriptors const &g_edge_descriptors, EdgeDescriptors const &h_edge_descriptors) const
// {
//   double result = 0;
//   // result = - sum_{edge e in G}
//   //              sum_{vertex i in H}
//   //                sum_{vertex j in H}
//   //                  p[e[0],i] * p[e[1],j] * abs(length(e) - length(i,j)) / (ng * nh)
//   for (auto edge_g_itr = esim->sourceEdgeIndex.cbegin(); edge_g_itr != esim->sourceEdgeIndex.cend(); ++edge_g_itr)
//   {
//     for (std::size_t i = 0; i < nh; ++i)
//     {
//       double length_e = std::get<3>(g_edge_descriptors.at(edge_g_itr->first.first, edge_g_itr->first.second));
//       for (std::size_t j = 0; j < nh; ++j)
//       {
//         double length_ij = std::get<3>(h_edge_descriptors.at(i, j));
//         result += p(edge_g_itr->first.first, i) * p(edge_g_itr->first.second, j) * abs(length_e - length_ij);
//       }
//     }
//   }
//   result /= x_len;    // result = result / (ng * nh)
//   return result;
// }

double GraphMatchingPath::f(double lambda, MatrixDouble const *p) const
{
  (void) lambda;
  (void) p;
  return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
}

void GraphMatchingPath::frankWolfe(double lambda, MatrixDouble *x_return, MatrixDouble const *x_start)
{
  // assert x basic feasible
  //memcpy(x, x_start->data(), x_len * sizeof(*x));
  //memcpy(y, x, x_len * sizeof(*x)); // y = x
  x.assign(x_start->data(), x_start->data() + x_len);
  y = x;

  double mu = 1.0;    // initialise with arbitrary nonzero
  double yDy = 1.0;   // initialise with arbitrary nonzero
  while (yDy != 0 && mu != 0) // TODO find correct stop criterion    //(zz != 0)
  {
    cprdbg::frankWolfe::print_x("x", x, nh, ng);   // debug
    cprdbg::frankWolfe::print_x("y", y, nh, ng);   // debug
    double xDy = simplex();  // y = argmax x^T D y, A y = 1, 0 <= y <= 1
    double xDx = mult_xD(x);
    yDy = bilinear(y, y);
    // be careful with sign of mu and maximize/minimize in lp??
    //mu = -(xDx - xDy) / (yDy - xDy - xDy + xDx); // at this point xDy is known.
    mu = frankWolfe::computeMu(xDx, xDy, yDy);
    //double mu = 0; // TODO solve for mu    // mu = (ww - wz) / (zz - wz - wz + ww);
    cprdbg::frankWolfe::print_bilinears(xDx, bilinear(x,x), xDy, bilinear(x,y), bilinear(y,x), yDy, bilinear(y,y), mu);
    //if (yDy - xDy - xDy + xDx == 0)   // if x == y, nothing to do
    //  mu = 0;
    //else
    if (mu >= 1.0)
      x = y;
    else if (mu < 0)
    {
      assert(yDy == 0);
      pcl::console::print_highlight("NEGATIVE MU!!!!!\n");
      pcl::console::print_info("    mu: %f\n", mu);
    }
    else
      for (std::size_t k = 0; k < x_len; ++k)
        x[k] = (1.0 - mu) * x[k] + mu * y[k];
  }
  memcpy(x_return->data(), y.data(), x_len * sizeof(double)); // TODO remove memcpy and make x_return->data() point to z
  //x_return = y;
}     // in their paper, frank-wolfe returns y instead of x, which seems consistent with their stop criterion.

// void GraphMatchingPath::setXTo1minusMuXPlusMuY(double mu)  // x = (1 - mu) * x + mu * y;
// {
//   for (std::size_t i = 0; i < x_len; ++i)
//     x[i] = (1.0 - mu) * x[i] + mu * y[i];
// }

#include <glpk.h>

void GraphMatchingPath::initSimplex(std::vector<int> const &iv, std::vector<int> const &jv, std::vector<double> const &av)
{
  // declare linear problem object
  lp = glp_create_prob();
  glp_set_prob_name(lp, "linear approximation");

  // declare objective: maximize
  glp_set_obj_dir(lp, GLP_MAX);

  // declare number of constraints and variables
  glp_add_rows(lp, nb_constraints);
  glp_add_cols(lp, x_len);

  // variables are nonnegative (and at most 1) and 1-indexed in glp
  for (std::size_t col = 1; col <= x_len; ++col)
    glp_set_col_bnds(lp, col, GLP_DB, 0.0, 1.0);  // 0 <= x[.] <= 1

  // load constraints
  cprdbg::frankWolfe::print_info_glploadmatrix(iv.size()-1);
  glp_load_matrix(lp, iv.size()-1, iv.data(), jv.data(), av.data());
  for (std::size_t row = 0; row < nb_constraints; ++row)
    glp_set_row_bnds(lp, row + 1, GLP_DB, 0.0, 1.0);    // row is 1-indexed
}

// update z to minimise W~ * Z
double GraphMatchingPath::simplex(void)
{
  static std::size_t nb_calls = 0;
  //std::cout << "\x1B[2J\x1B[H";   // non-portable hack to clear console
  //std::cout << "\x1B[H";           // non-portable hack to clear console
  pcl::console::print_highlight("simplex call %u\n\n", nb_calls);
  if (nb_calls > 20)
    exit(3);    // for debug
  ++nb_calls;

  // reset objective function
  // load objective function
  compute_lp_obj_coeffs(lp);

  cprdbg::frankWolfe::print_simplex(lp, ng, nh);

  // solve problem
  glp_simplex(lp, NULL);

  // update vector y with variables from the solution!!
  setYToSolutionOfLP(lp);

  // retrieve optimal objective value
  return (glp_get_obj_val(lp));
}


void GraphMatchingPath::compute_lp_obj_coeffs(glp_prob *lp)
{
  // start from 0
  //memset(xD, 0, x_len * sizeof(*xD));
  std::fill(xD.begin(), xD.end(), 0.0);
  std::cout << "This is xD("<<&xD<<") and it should all be zero:" << std::endl;
  cprdbg::frankWolfe::print_x("xD", xD, nh, ng);
  // add edge related rewards
  for (auto const &edge_g : esim->sourceEdgeIndex)
  {
    for (auto const &edge_h : esim->destEdgeIndex)
    {
      std::size_t kx = edge_g.first.first * nh + edge_h.first.first;
      std::size_t ky = edge_g.first.second * nh + edge_h.first.second;
      xD[ky] += x[kx] * esim->m(edge_g.second, edge_h.second);
      //glp_set_obj_coef(lp, ky+1, x[kx] * esim->m(edge_g.second, edge_h.second))
    }
  }
  // add vertex related rewards
  for (KeyT ig = 0; ig < ng; ++ig)
    for (KeyT ih = 0; ih < nh; ++ih)
      xD[ig * nh + ih] += x[ig * nh + ih] * (*vsim)(ig, ih);
      //glp_set_obj_coef(lp, ig * nh + ih + 1, x[ig * nh + ih] * (*vsim)(ig, ih));
  // send results to glpk
  for (std::size_t ky = 0; ky < x_len; ++ky)
    glp_set_obj_coef(lp, static_cast<int>(ky) + 1, xD[ky]);   // int 1-indexed

  cprdbg::frankWolfe::print_x_when_calculating_x_D(x, nh, ng);
  cprdbg::frankWolfe::print_x("objective coeffs", xD, nh, ng);
}

double GraphMatchingPath::mult_xD(std::vector<double> const &z) const
{
  double result = 0;
  for (std::size_t k = 0; k < x_len; ++k)
    result += xD[k] * z[k];

  cprdbg::frankWolfe::print_z_when_calculating_xD_z(result, z, nh, ng);

  return result;
}

// compute x D y
double GraphMatchingPath::bilinear(std::vector<double> const &x, std::vector<double> const &y) const
{
  double result = 0;
  for (auto const &edge_g : esim->sourceEdgeIndex)
  {
    for (auto const &edge_h : esim->destEdgeIndex)
    {
      std::size_t kx = edge_g.first.first * nh + edge_h.first.first;
      std::size_t ky = edge_g.first.second * nh + edge_h.first.second;
      result += x[kx] * y[ky] * esim->m(edge_g.second, edge_h.second);
    }
  }
  for (KeyT ig = 0; ig < ng; ++ig)
    for (KeyT ih = 0; ih < nh; ++ih)
      result += x[ig * nh + ih] * y[ig * nh + ih] * (*vsim)(ig, ih);

  cprdbg::frankWolfe::print_xz_when_calculating_x_D_z(result, x, y, nh, ng);

  return result; // = sum_(jg,jh) (sum_(ig,ih) x[ig,ih] D[(ig,ih)(jg,jh)]) y[jg,jh]
}

// set y to solution of solved lp
void GraphMatchingPath::setYToSolutionOfLP(glp_prob *lp)  // should be glp_prob const *lp but library wants it non-const for no reason
{
  for (std::size_t j = 0; j < x_len; ++j)
    y[j] = glp_get_col_prim(lp, static_cast<int>(j + 1));  // library wants int, 1-indexed
}

double frankWolfe::computeMu(double xDx, double xDy, double yDy)
{
  // mu maximizes quadratic formula v(mu) = (mu(y-x)+x)D(mu(y-x)+x) on interval [0,1]
  // hence either mu = 0, or mu = 1, or derivative v'(mu) at mu is 0
  // v'(mu) = 0  <==>  mu = (xDx-yDx) / (xDx-2xDy+yDy)
  double denominator = xDx - xDy - xDy + yDy;   // this should be negative or zero
  //assert (denominator <= 0);
  if (denominator == 0)
    return (1.0);

  double mu_suchthat_derivative_is_zero = (xDx - xDy) / denominator;

  std::vector<double> mu = {0, 1.0, mu_suchthat_derivative_is_zero};
  std::vector<double> v = {xDx, yDy, mu_suchthat_derivative_is_zero * (xDy - xDx) + xDx};
  // calculate three values of v: v(0), v(1), v(mu_suchthat_derivative_is_zero)
  double max_if_mu_is_zero = xDx;
  double max_if_mu_is_one = yDy;
  double max_if_derivative_at_mu_is_zero =
    mu_suchthat_derivative_is_zero * (xDy - xDx) + xDx;

  // return mu that maximizes v(mu)
  if (
      max_if_mu_is_zero > max_if_mu_is_one
      && max_if_mu_is_zero > max_if_derivative_at_mu_is_zero
  ) {
    return 0;
  }
  else if (max_if_mu_is_one > max_if_derivative_at_mu_is_zero)
    { return 1.0; }
  else
    { return mu_suchthat_derivative_is_zero; }
  // std::map<double, double, std::greater_than> mu;   // map v to mu
  // mu[xDx] = 0;
  // mu[yDy] = 1.0;
  // mu[mu_suchthat_derivative_is_zero * (xDy - xDx) + xDx] = mu_suchthat_derivative_is_zero;
  // return mu.cbegin()->second;  // return mu that maximizes v
}

// double frankWolfe::computeMu(double xDx, double xDy, double yDy)
// {
//   // mu maximizes quadratic formula v(mu) = (mu(y-x)+x)D(mu(y-x)+x) on interval [0,1]
//   // hence either mu = 0, or mu = 1, or derivative v'(mu) at mu is 0
//   // v'(mu) = 0  <==>  mu = (xDx-yDx) / (xDx-2xDy+yDy)
//   double denominator = xDx - xDy - xDy + yDy;   // this should be negative or zero
//   //assert (denominator <= 0);
//   if (denominator == 0)
//     return (1.0);
//
//   double mu_suchthat_derivative_is_zero = (xDx - xDy) / denominator;
//
//   // calculate three values of v: v(0), v(1), v(mu_suchthat_derivative_is_zero)
//   std::vector<double> mu = {0, 1.0, mu_suchthat_derivative_is_zero};
//   std::vector<double> v = {xDx, yDy, mu_suchthat_derivative_is_zero * (xDy - xDx) + xDx};
//
//   // return mu that maximizes v(mu)
//   return mu[
//     std::distance(
//       std::max_element(v.cbegin(), v.cend()),
//       v.cbegin()
//     )
//   ];
// }
