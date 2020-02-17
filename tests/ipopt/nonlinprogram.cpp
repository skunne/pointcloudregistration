#include <iostream>     /* output for GraphMatching::finalize_solution() */
#include <cassert>      /* assert(problem constants are what they should) */
#include <map>

#include <Eigen/Core>

#include <IpTNLP.hpp>

#include "nonlinprogram.hpp"

GraphMatching::GraphMatching(
  Eigen::MatrixXd const *v_sim,
  Eigen::MatrixXd const *e_sim,
  std::map<std::pair<Ipopt::Index, Ipopt::Index>, Ipopt::Index> const *srcEdgeIndex,
  std::map<std::pair<Ipopt::Index, Ipopt::Index>, Ipopt::Index> const *dstEdgeIndex)
  : nbnodes_src(v_sim->rows()), nbnodes_dst(v_sim->cols()),
    vertex_similarity(v_sim),
    sourceEdgeIndex(srcEdgeIndex), destEdgeIndex(dstEdgeIndex),
    edge_similarity(e_sim)
{
}

bool GraphMatching::get_nlp_info(
  Ipopt::Index&          n,
  Ipopt::Index&          m,
  Ipopt::Index&          nnz_jac_g,
  Ipopt::Index&          nnz_h_lag,
  Ipopt::TNLP::IndexStyleEnum& index_style
)
{
  n = nbnodes_src * nbnodes_dst;  // assignment: each pair (i_src, i_dst) should get 0 or 1
  m = nbnodes_src + nbnodes_dst;  // stochasticity: each row and each col sums to at most 1
  nnz_jac_g = 2 * n;
  nnz_h_lag = 10;
  index_style = Ipopt::TNLP::C_STYLE;

  return true;
}

bool GraphMatching::get_bounds_info(
  Ipopt::Index   n,
  Ipopt::Number* x_l,
  Ipopt::Number* x_u,
  Ipopt::Index   m,
  Ipopt::Number* g_l,
  Ipopt::Number* g_u
)
{
  assert(n==nbnodes_src * nbnodes_dst);
  assert(m==nbnodes_src + nbnodes_dst);
  /* forall xi, 0 <= xi <= 1 */
  for (Ipopt::Index i = 0; i < n; ++i)
    x_l[i] = 0.0;
  for (Ipopt::Index i = 0; i < n; ++i)
    x_u[i] = 1.0;
  /* 0 <= sum on each row and each line <= 1 */
  for (Ipopt::Index j = 0; j < m; ++j)
  {
    g_l[j] = 0.0;
    g_u[j] = 1.0;
  }
  return true;
}

bool GraphMatching::get_starting_point(
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
  assert(n==nbnodes_src * nbnodes_dst);
  (void) m;
  assert(init_x == true);
  assert(init_z == false);
  assert(init_lambda == false);
  if (init_x)
  {
    int min_dim = nbnodes_src < nbnodes_dst ? nbnodes_src : nbnodes_dst;
    x[0] = 1.0 / (Ipopt::Number) min_dim;
    for (Ipopt::Index i = 1; i < n; ++i)
      x[i] = x[0];
  }
  (void) z_L;   // ignore initial values for bound multipliers for absent bounds (xLi=−inf)
  (void) z_U;   // ignore initial values for bound multipliers for absent bounds (xUi=+inf)
  (void) lambda;
  return true;
}

/* compute xDx */
bool GraphMatching::eval_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number&       obj_value
)
{
  (void) new_x;
  assert(n==nbnodes_src * nbnodes_dst);

  obj_value = 0;
  for (auto const &edge_src : *sourceEdgeIndex)
  {
    for (auto const &edge_dst : *destEdgeIndex)
    {
      Ipopt::Index kx = edge_src.first.first * nbnodes_dst + edge_dst.first.first;
      Ipopt::Index ky = edge_src.first.second * nbnodes_dst + edge_dst.first.second;
      obj_value += x[kx] * x[ky] * (*edge_similarity)(edge_src.second, edge_dst.second);
    }
  }
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      obj_value += x[i_src * nbnodes_dst + i_dst] * x[i_src * nbnodes_dst + i_dst] * (*vertex_similarity)(i_src, i_dst);
  return (true);
}

bool GraphMatching::eval_grad_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number*       grad_f
)
{
  (void)  new_x;
  assert(n==nbnodes_src * nbnodes_dst);

  for (Ipopt::Index k = 0; k < n; ++k)
    grad_f[k] = 0;
  for (auto const &edge_src : *sourceEdgeIndex)
  {
    for (auto const &edge_dst : *destEdgeIndex)
    {
      Ipopt::Index kx = edge_src.first.first * nbnodes_dst + edge_dst.first.first;
      Ipopt::Index ky = edge_src.first.second * nbnodes_dst + edge_dst.first.second;
      grad_f[ky] += 2.0 * x[kx] * (*edge_similarity)(edge_src.second, edge_dst.second);
    }
  }
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      grad_f[i_src * nbnodes_dst + i_dst] += x[i_src * nbnodes_dst + i_dst] * (*vertex_similarity)(i_src, i_dst);
  return true;
}

bool GraphMatching::eval_g(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,      // unused parameter
  Ipopt::Index         m,
  Ipopt::Number*       g
)
{
  (void) new_x;   // don't use this parameter
  assert(n==nbnodes_src * nbnodes_dst);
  assert(m==nbnodes_src + nbnodes_dst);
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
  {
    g[i_src] = 0;
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      g[i_src] += x[i_src * nbnodes_dst + i_dst];
  }
  for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
  {
    g[nbnodes_src + i_dst] = 0;
    for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
      g[nbnodes_src + i_dst] += x[i_src * nbnodes_dst + i_dst];
  }
  return true;
}

bool GraphMatching::eval_jac_g(
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
  assert(n==nbnodes_src * nbnodes_dst);
  assert(m==nbnodes_src + nbnodes_dst);
  assert(nele_jac==2*n);
  (void) x;       // linear constraints => constant jacobian => x useless

  if (values != NULL)
  {
    for (Ipopt::Index j = 0; j < nele_jac; ++j)
      values[j] = 1.0;
  }
  else
  {
    Ipopt::Index j = 0;
    for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
    {
      for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      {
        iRow[j] = i_src;
        jCol[j] = i_src * nbnodes_dst + i_dst;
        ++j;
      }
    }
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
    {
      for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
      {
        iRow[j] = nbnodes_src + i_dst;
        jCol[j] = i_src * nbnodes_dst + i_dst;
        ++j;
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

bool GraphMatching::eval_h(
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

void GraphMatching::finalize_solution(
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
