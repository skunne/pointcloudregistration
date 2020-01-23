#include <iostream>   // std::cout
#include <vector>

#include "cpr_graphmatching_path.h"

namespace cprdbg
{
  namespace frankWolfe
  {
    void print_x(char const *name, std::vector<double> const &x, std::size_t width, std::size_t height)
    {
      std::cout << "Printing " << name << std::endl;
      for (std::size_t row = 0; row < height; ++row)
      {
        for (std::size_t col = 0; col < width; ++col)
          std::cout << x[row * width + col] << ' ';
        std::cout << std::endl;
      }
      assert(x.size() == width * height);
    }

    void print_xz_when_calculating_x_D_z(double result, std::vector<double> const &x, std::vector<double> const &z, std::size_t width, std::size_t height)
    {
      std::cout << "Calculating x*D*z="<<result<<"! here are current values of x("<< &x <<") and z("<< &z <<"):" << std::endl;
      print_x("x", x, width, height);
      print_x("z", z, width, height);
      std::cout << endl;
    }

    void print_x_when_calculating_x_D(std::vector<double> const &x, std::size_t width, std::size_t height)
    {
      std::cout << "Calculating x*D! here is current value of x("<< &x <<"):" << std::endl;
      print_x("x", x, width, height);
      std::cout << std::endl;
    }

    void print_z_when_calculating_xD_z(double result, std::vector<double> const &z, std::size_t width, std::size_t height)
    {
      std::cout << "Calculating xD*z="<<result<<"! here is current value of z("<< &z <<"):" << std::endl;
      print_x("z", z, width, height);
      std::cout << endl;
    }

    void print_bilinears(double xDx_1, double xDx_2, double xDy_1, double xDy_2, double yDx, double yDy_1, double yDy_2, double mu)
    {
      if (xDx_1 - xDx_2 < 0.00001 && xDy_1 - xDy_2 < 0.00001
          && yDx - xDy_2 < 0.00001 && yDy_1 - yDy_2 < 0.00001)
      {
        pcl::console::print_info("    xDx: %f\n", xDx_1);
        pcl::console::print_info("    xDy: %f\n", xDy_1);
        pcl::console::print_info("    yDy: %f\n", yDy_1);
        pcl::console::print_info("    mu:  %f\n", mu);
      }
      else
      {
        pcl::console::print_info("    xDx: %f == %f\n", xDx_1, xDx_2);
        pcl::console::print_info("    xDy: %f == %f\n", xDy_1, xDy_2);
        pcl::console::print_info("    yDx:           == %f\n", yDx);
        pcl::console::print_info("    yDy: %f == %f\n", yDy_1, yDy_2);
        pcl::console::print_info("    mu:  %f\n", mu);
        assert(xDx_1 - xDx_2 < 0.00001);
        assert(xDy_1 - xDy_2 < 0.00001);
        assert(yDx - xDy_2 < 0.00001);
        assert(yDy_1 - yDy_2 < 0.00001);
      }
    }

    void print_info_glploadmatrix(unsigned int nb_nonzero_coeffs)
    {
      pcl::console::print_info("glp_load_matrix with %u nonzero constraint coeffs.\n", nb_nonzero_coeffs);
    }

    void print_simplex(glp_prob *lp, int const ng, int const nh)
    {
      // print constraints
      pcl::console::print_info("constraints:\n");
      int ind[1+(ng<nh?nh:ng)];       // must be at least max(ng,nh)+1
      double coef[ng*nh+1];  // must be at least ng*nh+1
      ind[0] = 43; coef[0] = 43.0;  // cell [0] is never used because everything in the glp library is 1-indexed :-(
      for (int row = 1; row <= ng+nh; ++row)
      {
        int nb_nonzero = glp_get_mat_row(lp, row, ind, coef);
        int rowtype = glp_get_row_type(lp, row);
        double upperbound = glp_get_row_ub(lp, row);
        double lowerbound = glp_get_row_lb(lp, row);
        assert(ind[0] == 43);
        assert(coef[0] == 43.0);
        if ((row-1) < ng && nb_nonzero != nh)
          pcl::console::print_info("Wrong number of nonzero coeffs for constraint %d!! (has %d, should have %d = nh)\n", row, nb_nonzero, nh);
        else if ((row-1) >= ng && nb_nonzero != ng)
          pcl::console::print_info("Wrong number of nonzero coeffs for constraint %d!! (has %d, should have %d = ng)\n", row, nb_nonzero, ng);
        pcl::console::print_info("%.2f <= ", lowerbound);
        pcl::console::print_info("%.2f * x%02d", coef[1], ind[1]-1);
        for (int col = 2; col <= (row-1<ng?nh:ng); ++col)
          pcl::console::print_info(" + %.2f * x%02d", coef[col], ind[col]-1);
        assert(rowtype == GLP_DB);
        pcl::console::print_info(" <= %.2f\n", upperbound);
      }
      pcl::console::print_info("objective:\n");
      assert(glp_get_obj_dir(lp) == GLP_MAX);
      for (int j = 1; j <= ng*nh; ++j)
        coef[j] = glp_get_obj_coef(lp, j);
      pcl::console::print_info    ("Maximize  %.2f x%02d", coef[1], 0);
      int j = 2;
      for (int k = nh; k <= ng*nh; k += nh)
      {
        for (; j <= k; ++j)
          pcl::console::print_info(" + %.2f x%02d", coef[j], j-1);
        pcl::console::print_info("\n       ");
      }
      pcl::console::print_info("\n");
    }


  }
}
