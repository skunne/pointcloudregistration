
#ifndef __DEF_CPR_DEBUG_FRANKWOLFE_H__
# define __DEF_CPR_DEBUG_FRANKWOLFE_H__
#include "cpr_main.h"

namespace cprdbg
{
  namespace frankWolfe
  {
    int const verbosity = 1;
    
    void print_x(char const *name, std::vector<double> const &x,
      std::size_t width, std::size_t height, int verbosity);

    void print_xz_when_calculating_x_D_z(double result,
      std::vector<double> const &x, std::vector<double> const &z, std::size_t width,
      std::size_t height, int verbosity);

    void print_x_when_calculating_x_D(std::vector<double> const &x,
      std::size_t width, std::size_t height, int verbosity);
    void print_z_when_calculating_xD_z(double result, std::vector<double> const &z,
      std::size_t width, std::size_t height, int verbosity);

    //void print_bilinears(double xDx, double xDy, double yDy, double mu, GraphMatchingPath const *gm);
    //void print_bilinears(double xDx, double xDy, double yDy, double mu, double const *x, double const *y, double (GraphMatchingPath::*b)(double const *, double const *));
    void print_bilinears(double xDx_1, double xDx_2, double xDy_1, double xDy_2,
      double yDx, double yDy_1, double yDy_2, double mu, int verbosity);

    void print_info_glploadmatrix(unsigned int nb_nonzero_coeffs, int verbosity);

    void print_simplex(glp_prob *lp, int ng, int nh, int verbosity);
  }
}


#endif /* __DEF_CPR_DEBUG_FRANKWOLFE_H__ */
