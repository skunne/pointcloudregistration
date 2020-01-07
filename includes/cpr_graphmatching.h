
#ifndef __DEF_GRAPHMATCHING_H__
# define __DEF_GRAPHMATCHING_H__

#include "cpr_main.h"

/*
** cpr_graphmatching.cpp
*/

class GraphMatching
{
private:
  Eigen::MatrixXd const *vsim;    // vertex similarity matrix
  Eigen::MatrixXd const *esim;    // edge similarity matrix

  Eigen::MatrixXi const *g_adj;   // adjacency matrix of first graph
  Eigen::MatrixXi const *h_adj;   // adjacency matrix of second graph

  //Eigen::MatrixXd sol_concav;   // doubly stochastic
  //Eigen::MatrixXd sol_convex;   // doubly stochastic
  //Eigen::MatrixXd sol_smooth;   // doubly stochastic

public:
  Eigen::MatrixXi matching;          // permutation matrix
  // include matching as an std::map as well as a permutation matrix?

protected:
  double f_concav();
  double f_convex();
  double f_smooth();
  double f(double lambda, Eigen::MatrixXd const *p);
  frankWolfe(double lambda, Eigen::MatrixXd *p_new, Eigen::MatrixXd const *p_start);

public:
  GraphMatching(Eigen::MatrixXd const *vsim, Eigen::MatrixXd const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj);
  void run();
};

#endif /* __DEF_GRAPHMATCHING_H__ */
