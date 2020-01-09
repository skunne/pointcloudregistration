
#ifndef __DEF_GRAPHMATCHINGCGAL_H__
# define __DEF_GRAPHMATCHINGCGAL_H__

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
class GraphMatchingCgal : GraphMatching
{
private:
  //Eigen::MatrixXd const *vsim;    // vertex similarity matrix
  //EdgeSimilarityMatrix const *esim; // edge similarity matrix

  //Eigen::MatrixXi const *g_adj;   // adjacency matrix of first graph
  //Eigen::MatrixXi const *h_adj;   // adjacency matrix of second graph

  //Eigen::MatrixXd sol_concav;   // doubly stochastic, actually a permutation matrix
  //Eigen::MatrixXd sol_convex;   // doubly stochastic
  //Eigen::MatrixXd sol_smooth;   // doubly stochastic

public:
  //Eigen::MatrixXi matching;          // permutation matrix
  // include matching as an std::map as well as a permutation matrix?

protected:

public:
  GraphMatchingCgal(Eigen::MatrixXd const *vsim, EdgeSimilarityMatrix const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj);
  virtual void run();
  void fillMpsStream(std::stringstream &in) const;
  void fillQuadraticObjective(std::vector<int *> &D);       // objective, rowwise, on/below diagonal only, multiplied by a factor 2
  void fillStochasticityConstraints(std::vector<int *> &A); // make constraints, columnwise
  //unsigned int mappedVertex(unsigned int) const;
};

#endif /* __DEF_GRAPHMATCHINGCGAL_H__ */
