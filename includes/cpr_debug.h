
#ifndef __DEF_CPR_DEBUG_H__
# define __DEF_CPR_DEBUG_H__
#include "cpr_main.h"

namespace cprdbg
{
  namespace frankWolfe
  {
    void print_x(char const *name, std::vector<double> const &x, std::size_t width, std::size_t height);
    //void print_bilinears(double xDx, double xDy, double yDy, double mu, GraphMatchingPath const *gm);
    //void print_bilinears(double xDx, double xDy, double yDy, double mu, double const *x, double const *y, double (GraphMatchingPath::*b)(double const *, double const *));
    void print_bilinears(double xDx_1, double xDx_2, double xDy_1, double xDy_2, double yDx, double yDy_1, double yDy_2, double mu);
    void print_info_glploadmatrix(unsigned int nb_nonzero_coeffs);
    void print_simplex(glp_prob *lp, int ng, int nh);
  }
}


#endif /* __DEF_CPR_DEBUG_H__ */
