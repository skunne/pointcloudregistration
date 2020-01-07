#include "cpr_graphmatching.h"

GraphMatching::GraphMatching(Eigen::MatrixXd const *vsim, Eigen::MatrixXd const *esim, Eigen::MatrixXi const *g_adj, Eigen::MatrixXi const *h_adj)
  : vsim(vsim), esim(esim), g_adj(g_adj), h_adj(h_adj), matching(g_adj->rows(), h_adj->rows())
{
  // assert g_adj and h_adj are square matrices
}

void GraphMatching::run()
{

  // Initialisation
  double lambda;
  double lambda_new;
  Eigen::MatrixXd p(g_adj->rows(), h_adj->cols());
  Eigen::MatrixXd p_new(g_adj->rows(), h_adj->cols());

  lambda = 0;
  frankWolfe(lambda, &p, &p);

  double dlambda = 0.01;  // this is completely false, please change this
  double epsilon = 0.1;    // this is completely false, please change this

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
  // copy p into matching? but make sure this is p(1)?
}
