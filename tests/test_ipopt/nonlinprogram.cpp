#include <iostream>     /* output for NonLinProgram::finalize_solution() */
#include <cassert>      /* assert(problem constants are what they should) */

#include <IpTNLP.hpp>

#include "nonlinprogram.hpp"

bool NonLinProgram::get_nlp_info(
  Ipopt::Index&          n,
  Ipopt::Index&          m,
  Ipopt::Index&          nnz_jac_g,
  Ipopt::Index&          nnz_h_lag,
  Ipopt::TNLP::IndexStyleEnum& index_style
)
{
  n = 4;
  m = 2;
  nnz_jac_g = 8;
  nnz_h_lag = 10;
  index_style = Ipopt::TNLP::C_STYLE;

  return true;
}

bool NonLinProgram::get_bounds_info(
  Ipopt::Index   n,
  Ipopt::Number* x_l,
  Ipopt::Number* x_u,
  Ipopt::Index   m,
  Ipopt::Number* g_l,
  Ipopt::Number* g_u
)
{
  assert(n==4);
  assert(m==2);
  // forall xi, 1 <= xi <= 5
  for (Ipopt::Index i = 0; i < n; ++i)
    x_l[i] = 1.0;
  for (Ipopt::Index i = 0; i < n; ++i)
    x_u[i] = 5.0;
  // g0(x) >= 25
  g_l[0] = 25.0;
  g_u[0] = 2e19;  // +infinity (Ipopt::nlp_upperbound_inf ? Ipopt::ipopt_inf ?)
  // g1(x) == 40
  g_l[1] = g_u[1] = 40.0;

  return true;
}

bool NonLinProgram::get_starting_point(
  Ipopt::Index   n,
  bool    init_x,
  Ipopt::Number* x,
  bool    init_z,
  Ipopt::Number* z_L,
  Ipopt::Number* z_U,
  Ipopt::Index   m,
  bool    init_lambda,
  Ipopt::Number* lambda
)
{
  assert(n==4);
  (void) m;
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  if (init_x)
  {
    x[0] = 1.0;
    x[1] = 5.0;
    x[2] = 5.0;
    x[3] = 1.0;
  }
  (void) z_L;   // ignore initial values for bound multipliers for absent bounds (xLi=−inf)
  (void) z_U;   // ignore initial values for bound multipliers for absent bounds (xUi=+inf)
  (void) lambda;
  return true;
}

bool NonLinProgram::eval_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number&       obj_value
)
{
  (void) new_x;
  assert(n==4);
  obj_value = x[0] * x[3] * (x[0] + x[1] + x[2]) + x[2];
  return (true);
}

bool NonLinProgram::eval_grad_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number*       grad_f
)
{
  (void)  new_x;
  assert(n==4);
  grad_f[0] = x[3] * ((x[0] + x[1] + x[2]) + x[0]);
  grad_f[1] = x[0] * x[3];
  grad_f[2] = grad_f[1] + 1.0;
  grad_f[3] = x[0] * (x[0] + x[1] + x[2]);
  return true;
}

bool NonLinProgram::eval_g(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,      // unused parameter
  Ipopt::Index         m,
  Ipopt::Number*       g
)
{
  (void) new_x;   // don't use this parameter
  assert(n==4);
  assert(m==2);
  g[0] = x[0] * x[1] * x[2] * x[3];
  g[1] = x[0] * x[0] + x[1] * x[1] + x[2] * x[2] + x[3] * x[3];
  return true;
}

bool NonLinProgram::eval_jac_g(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,     // unused parameter
  Ipopt::Index         m,
  Ipopt::Index         nele_jac,
  Ipopt::Index*        iRow,
  Ipopt::Index*        jCol,
  Ipopt::Number*       values
)
{
  (void) new_x;   // don't use this parameter
  assert(n==4);
  assert(m==2);
  assert(nele_jac==8);
  if (values != NULL)
  {
    values[0] = x[1] * x[2] * x[3];
    values[1] = x[0] * x[2] * x[3];
    values[2] = x[0] * x[1] * x[3];
    values[3] = x[0] * x[1] * x[2];

    values[4] = x[0] + x[0];
    values[5] = x[1] + x[1];
    values[6] = x[2] + x[2];
    values[7] = x[3] + x[3];
  }
  else
  {
    Ipopt::Index k = 0;
    for (Ipopt::Index i = 0; i < m; ++i)
    {
      for (Ipopt::Index j = 0; j < n; ++j)
      {
        iRow[k] = i;
        jCol[k] = j;
        ++k;
      }
    }
  }
  // if (iRow != NULL)
  // {
  //   for (Ipopt::Index i = 0; i < n; ++i)
  //     iRow[i] = 0;
  //   for (Ipopt::Index i = n; i < 2 * n; ++i)
  //     iRow[i] = 1;
  // }
  // if (jCol != NULL)
  // {
  //   for (Ipopt::Index i = 0; i < n; ++i)
  //   {
  //     jCol[i] = i;
  //     jCol[i+n] = i;
  //   }
  // }
  return (true);
}

bool NonLinProgram::eval_h(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool                 new_x,      // unused parameter
   Ipopt::Number        obj_factor,
   Ipopt::Index         m,
   const Ipopt::Number* lambda,     // unused parameter
   bool          new_lambda,
   Ipopt::Index         nele_hess,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
)
{
  (void) new_x;        // don't use this parameter
  (void) new_lambda;   // don't use this parameter
  assert(n==4);
  assert(m==2);
  if (values != NULL)
  {
    // objective terms
    values[0] = 0;

    values[1] = obj_factor * x[2] * x[3];
    values[2] = 0;

    values[3] = obj_factor * x[1] * x[3];
    values[4] = obj_factor * x[0] * x[3];
    values[5] = 0;

    values[6] = obj_factor * x[1] * x[2];
    values[7] = obj_factor * x[0] * x[2];
    values[8] = obj_factor * x[0] * x[1];
    values[9] = 0;

    // constraint g0 terms
    values[0] += 0;

    values[1] += lambda[0] * x[2] * x[3];
    values[2] += 0;

    values[3] += lambda[0] * x[1] * x[3];
    values[4] += lambda[0] * x[0] * x[3];
    values[5] += 0;

    values[6] += lambda[0] * x[1] * x[2];
    values[7] += lambda[0] * x[0] * x[2];
    values[8] += lambda[0] * x[0] * x[1];
    values[9] += 0;

    // constraint g1 terms
    values[0] += lambda[1] * 2.0;

    values[1] += 0;
    values[2] += lambda[1] * 2.0;

    values[3] += 0;
    values[4] += 0;
    values[5] += lambda[1] * 2.0;

    values[6] += 0;
    values[7] += 0;
    values[8] += 0;
    values[9] += lambda[1] * 2.0;
  }
  else
  {
    Ipopt::Index k = 0;
    for (Ipopt::Index i = 0; i < 4; ++i)
    {
      for (Ipopt::Index j = 0; j <= i; ++j)
      {
        iRow[k] = i;
        jCol[k] = j;
        ++k;
      }
    }
    assert(k==nele_hess);
  }
  return true;
}

void NonLinProgram::finalize_solution(
    Ipopt::SolverReturn               status,
    Ipopt::Index                      n,
    const Ipopt::Number*              x,
    const Ipopt::Number*              z_L,
    const Ipopt::Number*              z_U,
    Ipopt::Index                      m,
    const Ipopt::Number*              g,
    const Ipopt::Number*              lambda,
    Ipopt::Number                     obj_value,
    const Ipopt::IpoptData*           ip_data,
    Ipopt::IpoptCalculatedQuantities* ip_cq
  )
  {
    (void) status;
    (void) lambda;
    (void) ip_data;
    (void) ip_cq;
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.
    // For this example, we write the solution to the console
    std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
    for( Ipopt::Index i = 0; i < n; i++ )
    {
       std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }
    std::cout << std::endl << std::endl << "Solution of the bound multipliers, z_L and z_U" << std::endl;
    for( Ipopt::Index i = 0; i < n; i++ )
    {
       std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    }
    for( Ipopt::Index i = 0; i < n; i++ )
    {
       std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl;
    }
    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;
    std::cout << std::endl << "Final value of the constraints:" << std::endl;
    for( Ipopt::Index i = 0; i < m; i++ )
    {
       std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
  }
