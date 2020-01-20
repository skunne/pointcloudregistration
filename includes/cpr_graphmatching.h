
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
protected:
  MatrixDouble const *vsim;    // vertex similarity matrix
  EdgeSimilarityMatrix const *esim; // edge similarity matrix

  MatrixInt const *g_adj;   // adjacency matrix of first graph
  MatrixInt const *h_adj;   // adjacency matrix of second graph

  //MatrixDouble sol_concav;   // doubly stochastic, actually a permutation matrix
  //MatrixDouble sol_convex;   // doubly stochastic
  //MatrixDouble sol_smooth;   // doubly stochastic

public:
  MatrixInt matching;          // permutation matrix
  // include matching as an std::map as well as a permutation matrix?

protected:
  //double f_concav() const;
  //double f_convex() const;
  //double f_smooth(MatrixDouble const *p) const;
  //double f(double lambda, MatrixDouble const *p) const;
  //void frankWolfe(double lambda, MatrixDouble *p_new, MatrixDouble const *p_start);

public:
  GraphMatching(MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim, MatrixInt const *g_adj, MatrixInt const *h_adj);
  virtual void run();
  //unsigned int mappedVertex(unsigned int) const;
};

#endif /* __DEF_GRAPHMATCHING_H__ */
