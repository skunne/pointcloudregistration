
#ifndef __DEF_GRAPHMATCHINGPATH_H__
# define __DEF_GRAPHMATCHINGPATH_H__

#include <sstream>
#include <glpk.h>   // linear programming solver
#include "cpr_main.h"
#include "cpr_matrices.h"
#include "cpr_graphmatching.h"

/**
A graph matching algorithm inspired by Zaslavskiy and Huang

	@author Stephan Kunne <stephan.kunne@univ-nantes.fr>
*/
/*
** cpr_graphmatching.cpp
*/
class GraphMatchingPath : GraphMatching
{
private:
  //MatrixDouble const *vsim;    // vertex similarity matrix
  //EdgeSimilarityMatrix const *esim; // edge similarity matrix

  //MatrixInt const *g_adj;   // adjacency matrix of first graph
  //MatrixInt const *h_adj;   // adjacency matrix of second graph

  //MatrixDouble sol_concav;   // doubly stochastic, actually a permutation matrix
  //MatrixDouble sol_convex;   // doubly stochastic
  //MatrixDouble sol_smooth;   // doubly stochastic

protected:
  std::size_t ng;
  std::size_t nh;
  std::size_t x_len;  // nb var = ng * nh
  std::size_t nb_constraints; // ng + nh (stochasticity constraints)

  glp_prob *lp;
  std::vector<int> cons_coeff_rowindex;  // nonzero constraint coeff rowindex
  std::vector<int> cons_coeff_colindex;  // nonzero constraint coeff colindex
  std::vector<double> cons_coeff_value;  // nonzero constraint coeff value

  double *x;
  double *y;   // feasible vector for PII in frank-wolfe
  double *xD;   // coeffs of linear objective when x is fixed

protected:
  //double f_concav() const;
  //double f_convex() const;
  double f_smooth(MatrixDouble const *p) const;
  double f(double lambda, MatrixDouble const *p) const;
  //void frankWolfe(double lambda, MatrixDouble *x_return, MatrixDouble const *x_start);
  //double simplex(void);   // y = argmax 2 x^T D y, A y = 1, 0 <= y <= 1
  void initSimplex(std::vector<int> const &iv, std::vector<int> const &jv, std::vector<double> const &av);
  double simplex(void);
  void compute_lp_obj_coeffs(glp_prob *lp);  // compute lp objective function coefficients
  double mult_xD(double const *z) const;   // compute xDz reusing xD coeffs stored in variable xD
  void setYToSolutionOfLP(glp_prob *lp);  // set y to solution of solved lp
  void setXTo1minusMuXPlusMuY(double mu);  // x = (1.0 - mu) * x + mu * y;

public:
  void frankWolfe(double lambda, MatrixDouble *x_return, MatrixDouble const *x_start);
  double bilinear(double const *x, double const *y) const; // recompute xDy
  GraphMatchingPath(MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj);
  ~GraphMatchingPath();
  virtual void run();
  //unsigned int mappedVertex(unsigned int) const;
};

namespace frankWolfe
{
  double computeMu(double xDx, double xDy, double yDy);
}

#endif /* __DEF_GRAPHMATCHINGPATH_H__ */
