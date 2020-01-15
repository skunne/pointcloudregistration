
#ifndef __DEF_GRAPHMATCHINGPATH_H__
# define __DEF_GRAPHMATCHINGPATH_H__

#include <sstream>
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
  //Eigen::MatrixXd const *vsim;    // vertex similarity matrix
  //EdgeSimilarityMatrix const *esim; // edge similarity matrix

  //Eigen::MatrixXi const *g_adj;   // adjacency matrix of first graph
  //Eigen::MatrixXi const *h_adj;   // adjacency matrix of second graph

  //Eigen::MatrixXd sol_concav;   // doubly stochastic, actually a permutation matrix
  //Eigen::MatrixXd sol_convex;   // doubly stochastic
  //Eigen::MatrixXd sol_smooth;   // doubly stochastic

protected:
  double *x;
  double *y;   // feasible vector for PII in frank-wolfe
  std::size_t *base;    // base indices for simplex
  std::size_t *nonbase; // nonbase indices for simplex
  double *reduced_cost;  // reduced cost of each variable for simplex
  std::size_t x_len;  // nb var = ng * nh
  std::size_t n;    // assume ng = nh??
  std::size_t nb_constraints; // ng + nh (stochasticity constraints)

protected:
  double f_concav() const;
  double f_convex() const;
  double f_smooth(Eigen::MatrixXd const *p) const;
  double f(double lambda, Eigen::MatrixXd const *p) const;
  void frankWolfe(double lambda, Eigen::MatrixXd *x_return, Eigen::MatrixXd const *x_start);
  double simplex(void);   // y = argmax 2 x^T D y, A y = 1, 0 <= y <= 1
  void initSimplex(void);
  void updateX(double mu);  // x = (1.0 - mu) * x + mu * y;

public:
  GraphMatchingPath(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj);
  ~GraphMatchingPath();
  virtual void run();
  //unsigned int mappedVertex(unsigned int) const;
};

#endif /* __DEF_GRAPHMATCHINGPATH_H__ */
