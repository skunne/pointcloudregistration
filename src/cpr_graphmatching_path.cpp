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
  u_len = ng + nh;
  y_len = ng + nh;    // nb constraints
  v_len = x_len;
  z = (double *) malloc((x_len + u_len + y_len + v_len) * sizeof(*z));
  w = (double *) malloc((x_len + u_len + y_len + v_len) * sizeof(*w));
  // base = malloc(realnbconstraints * sizeof(std::size_t));
  // nonbase = malloc((realnbvar - realnbconstraints) * sizeof(std::size_t));
  // reduced_cost = malloc(realnbconstraints * sizeof(double));
}

GraphMatchingPath::~GraphMatchingPath()
{
  free(z);
  free(w);
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
  // Phase I
  //   find U,Y,V such that
  //   Z = [X U Y V] is solution of PII
  //     if no solution: ??
  //   W = Z

  // BREAKING NEWS actually y = 0 always
  // for phase I, we can always choose u = 0 and v = 0 ???

  // memcpy(z, x_start->data(), x_len * sizeof(double));    // good news: Eigen::MatrixXd::data() is column major
  completeBasicFeasibleZ(x_start);
  memcpy(w, z, (x_len + u_len + y_len + v_len) * sizeof(*z)); // w = z

  // Phase II
  //   loop:
  //     apply simplex method to maximize (- W~ * Z)
  //       stop simplex when one of the two conditions is true:
  //         if Z~ * Z == 0:
  //           halt and return solution Z
  //         if W~ * Z <= 1/2 W~ W:
  //           mu = W~ * (W - Z) / ((Z~ - W~)*(Z - W))
  //           mu = min(mu, 1)
  //           W = W + mu (Z - W)


  initSimplex();      // initialise reduced cost with coeffs from w
  double wz;          // dot products W~ * Z, W~ * W and Z~ * Z
  double ww;
  double zz = 1.0;    // initialize with arbitrary nonzero value
  while (zz != 0)
  {
    wz = nextSimplexStep();  // update z to maximize  (- W~ * Z)
    zz = adjointMult(z,z);
    ww = adjointMult(w, w);
    if (wz + wz <= ww)
    {
      // update w so that new w maximizes new objective on segment [w, z]
      double mu = (ww - wz) / (zz - wz - wz + ww);
      if (mu >= 1.0)
        memcpy(w, z, (x_len + u_len + y_len + x_len) * sizeof(*z)); // w = z
      else
        updateW(mu); // w = (1 - mu) * w + mu * z;
      initSimplex(); // update reduced_cost according to new w
    }
  }
  memcpy(x_return->data(), z, x_len * sizeof(double)); // TODO remove memcpy and make x_return->data() point to z
}

// return scalar product a~ * b where [x u y v]~ = [v y u x]
double GraphMatchingPath::adjointMult(double const *a, double const *b) const
{
  double const *const xa = &a[0];
  double const *const ua = &a[x_len];
  double const *const ya = &a[x_len + u_len];         // BREAKING NEWS ya = 0 if a feasible
  double const *const va = &a[x_len + u_len + y_len];
  double const *const xb = &b[0];
  double const *const ub = &b[x_len];
  double const *const yb = &b[x_len + u_len];         // BREAKING NEWS yb = 0 if b feasible
  double const *const vb = &b[x_len + u_len + y_len];
  double result = 0;
  for (std::size_t i = 0; i < x_len; ++i)
  {
    result += va[i] * xb[i];
    result += xa[i] * vb[i];
  }
  for (std::size_t j = 0; j < u_len; ++j)
  {
    result += ya[j] * ub[j];   // = 0 if a feasible
    result += ua[j] * yb[j];   // = 0 if b feasible
  }
  return result;
}

void GraphMatchingPath::updateW(double mu)  // w = (1 - mu) * w + mu * z;
{
  for (std::size_t i = 0; i < x_len + u_len + y_len + v_len; ++i)
    w[i] = (1.0 - mu) * w[i] + mu * z[i];
}

void GraphMatchingPath::completeBasicFeasibleZ(Eigen::MatrixXd const *x_start)
{
  //   find U,Y,V such that
  //   Z = [X U Y V] is solution of PII
  //     if no solution: ??
  memcpy(z, x_start->data(), x_len * sizeof(double)); // x=x // TODO remove memcpy and make x point to z[0]
  // BREAKING NEWS actually y = 0 always
  // for phase I, we can always choose u = 0 and v = 0 ???
  memset(&z[x_len], 0, (u_len + y_len + v_len) * sizeof(*z));  // u=0 y=0 v=0
  // TODO ASSERT z REALLY IS FEASIBLE AND BASIC
}

// update z to minimise W~ * Z
double GraphMatchingPath::nextSimplexStep(void)
{
  // find pivot i such that reduced_cost[i] < 0 (for instance i that minimises reduced_cost[i])
  std::size_t i = 0;
  while (i < x_len + u_len + y_len + v_len && reduced_cost[i] >= 0)
    ++i;
  assert (i < x_len + u_len + y_len + v_len); // there exists iw such that w[iw] < 0
  assert(i < x_len || i >= x_len + u_len + y_len); // cannot be in u or y since y = 0



  // find constraint j such that constraint j exploses first when z[i] is increased

  // update z[i] with appropriate new value

  // find basic variable k corresponding to constraint j; set z[k] = 0

  // update other basic variables
  // update residual cost

  return adjointMult(w, z);
}

// initialise the "reduced cost" of each variable for the simplex algorithm
void GraphMatchingPath::initSimplex(void)
{
  memcpy(reduced_cost, &w[x_len + u_len + y_len], x_len * sizeof(*reduced_cost));  // x_rc = v_w
  memcpy(&reduced_cost[x_len + u_len + y_len], w, v_len * sizeof(*reduced_cost));  // v_rc = x_w
  memset(&reduced_cost[x_len], 0, (u_len + y_len) * sizeof(*reduced_cost));   // u_rc = 0 , y_rc = 0
}
