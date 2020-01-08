
#ifndef __DEF_GRAPHMATCHING_H__
# define __DEF_GRAPHMATCHING_H__

#include "cpr_main.h"
#include "cpr_matrices.h"

/**
A graph matching algorithm inspired by Zaslavskiy and Huang

	@author Stephan Kunne <stephan.kunne@univ-nantes.fr>
*/
/*
** cpr_graphmatching.cpp
*/
class GraphMatching
{
private:
  Eigen::MatrixXd const *vsim;    // vertex similarity matrix
  EdgeSimilarityMatrix const *esim; // edge similarity matrix

  Eigen::MatrixXi const *g_adj;   // adjacency matrix of first graph
  Eigen::MatrixXi const *h_adj;   // adjacency matrix of second graph

  //Eigen::MatrixXd sol_concav;   // doubly stochastic, actually a permutation matrix
  //Eigen::MatrixXd sol_convex;   // doubly stochastic
  //Eigen::MatrixXd sol_smooth;   // doubly stochastic

public:
  Eigen::MatrixXi matching;          // permutation matrix
  // include matching as an std::map as well as a permutation matrix?

protected:
  double f_concav() const;
  double f_convex() const;
  double f_smooth(Eigen::MatrixXd const *p) const;
  double f(double lambda, Eigen::MatrixXd const *p) const;
  void frankWolfe(double lambda, Eigen::MatrixXd *p_new, Eigen::MatrixXd const *p_start);

public:
  GraphMatching(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj);
  void run();
  //unsigned int mappedVertex(unsigned int) const;
};

#endif /* __DEF_GRAPHMATCHING_H__ */
