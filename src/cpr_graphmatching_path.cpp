//#include <cassert>        // assert(eg == esim.sourceEdgeIndex.at(edge_g_itr->first));
//#include <stdexcept>    // std::out_of_range

// #include <CGAL/QP_models.h>     // quadratic programming
// #include <CGAL/QP_functions.h>
// #include <CGAL/MP_Float.h>
// typedef CGAL::MP_Float ET;

#include <cstdlib>  // malloc, free
#include <cstring>  // memcpy, memset

#include "cpr_graphmatching.h"
#include "cpr_graphmatching_path.h"

GraphMatchingPath::GraphMatchingPath(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
  : GraphMatching(vsim, esim, g_adj, h_adj)
{
  std::size_t ng = g_adj->rows();
  std::size_t nh = h_adj->rows();
  x_len = ng * nh;    // nb variables
  nb_constraints = ng + nh;    // nb constraints
  assert(ng = nh);    // is that necessary?
  n = ng;
  x = (double *) malloc(x_len * sizeof(*x));
  y = (double *) malloc(x_len * sizeof(*y));
  // base = malloc(realnbconstraints * sizeof(std::size_t));
  // nonbase = malloc((realnbvar - realnbconstraints) * sizeof(std::size_t));
  // reduced_cost = malloc(realnbconstraints * sizeof(double));
}

GraphMatchingPath::~GraphMatchingPath()
{
  free(x);
  free(y);
  free(base);
  free(nonbase);
  free(reduced_cost);
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
  // when lambda == 1, all coeffs in p must be 0.0 or 1.0
  // copy p into this->matching and convert double to int
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
  res /= x_len;   // res = res / (ng * nh)
  return res;
}

double GraphMatchingPath::f(double lambda, Eigen::MatrixXd const *p) const
{
  (void) lambda;
  (void) p;
  return 0;   // TODO probably an accessor to an attribute that was updated by frankWolfe()
}

void GraphMatchingPath::frankWolfe(double lambda, Eigen::MatrixXd *x_return, Eigen::MatrixXd const *x_start)
{
  // assert x basic feasible
  memcpy(y, x, x_len * sizeof(*x)); // y = x

  while (1) // TODO find correct stop criterion    //(zz != 0)
  {
    simplex();  // y = argmax 2 x^T D y, A y = 1, 0 <= y <= 1

    double mu = 0;//(ww - wz) / (zz - wz - wz + ww);
    if (mu >= 1.0)
      memcpy(x, y, x_len * sizeof(*y)); // x = y
    else
      updateX(mu); // x = (1 - mu) * x + mu * y;
  }
  memcpy(x_return->data(), x, x_len * sizeof(double)); // TODO remove memcpy and make x_return->data() point to z
}

void GraphMatchingPath::updateX(double mu)  // x = (1 - mu) * x + mu * y;
{
  for (std::size_t i = 0; i < x_len; ++i)
    x[i] = (1.0 - mu) * x[i] + mu * y[i];
}

// update z to minimise W~ * Z
double GraphMatchingPath::simplex(void)
{
  double obj;       // value of objective function
  initSimplex();   // init reduced cost

  // find pivot i such that reduced_cost[i] < 0 (for instance i that minimises reduced_cost[i])
  std::size_t i = 0;
  while (i < x_len && (reduced_cost[i] >= 0 || z[i] > 0))   // browse all variables, not good
  while (i < nonbase_len && reduced_cost[nonbase[i]] >= 0)  // browse nonbasic variables only
    ++i;
  if (i == x_len)   // no negative reduced cost: solution is optimal
    return obj;

  // find constraint j such that constraint j exploses first when z[i] is increased
  std::size_t j = 0;
  double ratio = ?? / ??; // TODO find better variable name
  for (std::size_t newj = 0; newj < nb_constraints; ++newj)
  {
    if ((newratio = ?? / ??) < ratio)
    {
      ratio = newratio;
      j = newj;
    }
    // PAS BESOIN DE FAIRE UNE BOUCLE POUR TROUVER J
    // ON EST INTELLIGENT ET LES CONTRAINTES ONT UNE LOGIQUE
  }

  // update z[i] with appropriate new value
  z[i] = ??;

  // find basic variable k corresponding to constraint j; set z[k] = 0
  z[base[j]] = 0;
  nonbase[??] = base[j];  // exit z[k]
  base[j] = i;            // enter z[i]

  // update other basic variables
  // update residual cost

  return obj;
}

// initialise the "reduced cost" of each variable for the simplex algorithm
void GraphMatchingPath::initSimplex(void)
{
  memset(reduced_cost, 0, x_len * sizeof(*reduced_cost));

  // reduced_cost[edge eg] = sum_(edge eh) esim(eg,eh) * x[eh]
  for (auto edge_g_itr = esim->sourceEdgeIndex.cbegin(); edge_g_itr != esim->sourceEdgeIndex.cend(); ++edge_g_itr)
  {
    std::size_t k = edge_g_itr->first.first * n + edge_g_itr->first.second;
    for (auto edge_h_itr = esim->destEdgeIndex.cbegin(); edge_h_itr != esim->destEdgeIndex.cend(); ++edge_h_itr)
      reduced_cost[k] += esim->m(edge_g_itr->second, edge_h_itr->second) * x[edge_h_itr->first.first * n + edge_h_itr->first.second];
  }

  // reduced_cost[vertex ig] = sum_(vertex ih) vsim(ig,ih) * x[ig,ih]
  for (std::size_t ig = 0; ig < n; ++ig)
  {
    for (std::size_t ih = 0; ih < n; ++ih)
      reduced_cost[ig * (n + 1)] += (*vsim)(ig, ih) * x[ig * n + ih];
  }

  // reduced_cost[not an edge nor a vertex] = 0
}
