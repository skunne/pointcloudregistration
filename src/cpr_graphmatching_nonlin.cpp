#include <iostream>     /* output for GMNonlinProblem::finalize_solution() */
#include <cassert>      /* assert(problem constants are what they should) */
#include <map>

#include <Eigen/Core>

#include <IpIpoptApplication.hpp>   // IpoptApplication in GMNonlinProblem::run()
#include <IpTNLP.hpp>               // parent class of GMNonlinProblem

#include "cpr_graphmatching_nonlin.h"

GMNonlinProblem::GMNonlinProblem(
  MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
  MatrixInt const *g_adj, MatrixInt const *h_adj
)
  : GraphMatching(vsim, esim, g_adj, h_adj),
    nbnodes_src(vsim->rows()), nbnodes_dst(vsim->cols())
{
  //std::cout << "    METHOD GMNonlinProblem::GMNonlinProblem()" << std::endl;
  buildHessian();
}

GraphMatchingNonlin::GraphMatchingNonlin(
  MatrixDouble const *_vsim, EdgeSimilarityMatrix const *_esim,
  MatrixInt const *_g_adj, MatrixInt const *_h_adj
)
  : GraphMatching(_vsim, _esim, _g_adj, _h_adj),
    problem(new GMNonlinProblem(_vsim, _esim, _g_adj, _h_adj))
{
}

void GMNonlinProblem::buildHessian(void)
{
  //std::cout << "    METHOD GMNonlinProblem::BUILDHESSIAN()" << std::endl;
  //std::cout << "      dim of e_sim: (" << edge_similarity->rows()<<','<<edge_similarity->cols() << ")" << std::endl;
  /* give only lower triangular entries of 2D, which is symmetric */

  /* strictly lower triangular */
  for (auto const &edge_src : esim->sourceEdgeIndex)
  {
    for (auto const &edge_dst : esim->destEdgeIndex)
    {
      Ipopt::Index kx = edge_src.first.first * nbnodes_dst + edge_dst.first.first;
      Ipopt::Index ky = edge_src.first.second * nbnodes_dst + edge_dst.first.second;
      if (kx > ky)  // TODO find smarter way to test kx > ky
      {
        hessian_iRow.push_back(kx);
        hessian_jCol.push_back(ky);
        //std::cout << "        ("<< edge_src.second << ',' << edge_dst.second <<")" << std::endl;
        hessian_values.push_back(2.0 * (esim->m)(edge_src.second, edge_dst.second));
      }
    }
  }

  //std::cout << "      dim of v_sim: (" << vertex_similarity->rows()<<','<<vertex_similarity->cols() << ")" << std::endl;
  /* diagonal */
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
  {
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
    {
      hessian_iRow.push_back(i_src * nbnodes_dst + i_dst);
      hessian_jCol.push_back(hessian_iRow.back());
      //std::cout << "        ("<< i_src << ',' << i_dst <<")" << std::endl;
      hessian_values.push_back(2.0 * (*vsim)(i_src,i_dst));
    }
  }
}

bool GMNonlinProblem::get_nlp_info(
  Ipopt::Index&          n,
  Ipopt::Index&          m,
  Ipopt::Index&          nnz_jac_g,
  Ipopt::Index&          nnz_h_lag,
  Ipopt::TNLP::IndexStyleEnum& index_style
)
{
  //std::cout << "    METHOD GMNonlinProblem::GET_NLP_INFO()" << std::endl;
  n = nbnodes_src * nbnodes_dst;  // assignment: each pair (i_src, i_dst) should get 0 or 1
  m = nbnodes_src + nbnodes_dst;  // stochasticity: each row and each col sums to at most 1
  nnz_jac_g = 2 * n;
  assert(hessian_values.size() == hessian_iRow.size() && hessian_values.size() == hessian_jCol.size());
  nnz_h_lag = hessian_values.size();
  index_style = Ipopt::TNLP::C_STYLE;

  return true;
}

bool GMNonlinProblem::get_bounds_info(
  Ipopt::Index   n,
  Ipopt::Number* x_l,
  Ipopt::Number* x_u,
  Ipopt::Index   m,
  Ipopt::Number* g_l,
  Ipopt::Number* g_u
)
{
  //std::cout << "    METHOD GMNonlinProblem::GET_BOUNDS_INFO()" << std::endl;
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

bool GMNonlinProblem::get_starting_point(
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
  //std::cout << "    METHOD GMNonlinProblem::GET_STARTING_POINT()" << std::endl;
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
bool GMNonlinProblem::eval_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number&       obj_value
)
{
  //std::cout << "    METHOD GMNonlinProblem::EVAL_F()" << std::endl;
  (void) new_x;
  assert(n==nbnodes_src * nbnodes_dst);

  obj_value = 0;
  for (auto const &edge_src : esim->sourceEdgeIndex)
  {
    for (auto const &edge_dst : esim->destEdgeIndex)
    {
      Ipopt::Index kx = edge_src.first.first * nbnodes_dst + edge_dst.first.first;
      Ipopt::Index ky = edge_src.first.second * nbnodes_dst + edge_dst.first.second;
      obj_value += x[kx] * x[ky] * (esim->m)(edge_src.second, edge_dst.second);
    }
  }
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      obj_value += x[i_src * nbnodes_dst + i_dst] * x[i_src * nbnodes_dst + i_dst] * (*vsim)(i_src, i_dst);
  return (true);
}

/* compute jac(xDx) = 2xD */
bool GMNonlinProblem::eval_grad_f(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,
  Ipopt::Number*       grad_f
)
{
  //std::cout << "    METHOD GMNonlinProblem::EVAL_GRAD_F()" << std::endl;
  (void)  new_x;
  assert(n==nbnodes_src * nbnodes_dst);

  for (Ipopt::Index k = 0; k < n; ++k)
    grad_f[k] = 0;
  for (auto const &edge_src : esim->sourceEdgeIndex)
  {
    for (auto const &edge_dst : esim->destEdgeIndex)
    {
      Ipopt::Index kx = edge_src.first.first * nbnodes_dst + edge_dst.first.first;
      Ipopt::Index ky = edge_src.first.second * nbnodes_dst + edge_dst.first.second;
      grad_f[ky] += 2.0 * x[kx] * (esim->m)(edge_src.second, edge_dst.second);
    }
  }
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
      grad_f[i_src * nbnodes_dst + i_dst] += x[i_src * nbnodes_dst + i_dst] * (*vsim)(i_src, i_dst);
  return true;
}

/* compute sum(each row of x) and sum(each col of x) */
bool GMNonlinProblem::eval_g(
  Ipopt::Index         n,
  const Ipopt::Number* x,
  bool                 new_x,      // unused parameter
  Ipopt::Index         m,
  Ipopt::Number*       g
)
{
  //std::cout << "    METHOD GMNonlinProblem::EVAL_G()" << std::endl;
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

/* linear constraints => constant jacobian */
bool GMNonlinProblem::eval_jac_g(
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
  //std::cout << "    METHOD GMNonlinProblem::EVAL_JAC_G()" << std::endl;
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
  return (true);
}

/* compute sigma hessian(objective) + sum_(constraint j) lambda_j hessian(constraint j) = 2 sigma D + 0 */
bool GMNonlinProblem::eval_h(
   Ipopt::Index         n,
   const Ipopt::Number* x,
   bool                 new_x,      // unused parameter
   Ipopt::Number        obj_factor,
   Ipopt::Index         m,
   const Ipopt::Number* lambda,
   bool          new_lambda,     // unused parameter
   Ipopt::Index         nele_hess,
   Ipopt::Index*        iRow,
   Ipopt::Index*        jCol,
   Ipopt::Number*       values
)
{
  //std::cout << "    METHOD GMNonlinProblem::EVAL_H()" << std::endl;
  (void) new_x;        // don't use this parameter
  (void) new_lambda;   // don't use this parameter
  assert(n==nbnodes_src * nbnodes_dst);
  assert(m==nbnodes_src + nbnodes_dst);
  (void) x;  // quadratic obj + linear constraints => constant hessian => x useless
  (void) lambda; // linear constraints => constraint terms zero in hessian => lambda useless
  assert(nele_hess == hessian_values.size());
  if (values != NULL)
  {
    for (std::size_t j = 0; j < hessian_values.size(); ++j)
      values[j] = obj_factor * hessian_values[j];
  }
  else
  {
    for (std::size_t j = 0; j < hessian_iRow.size(); ++j)
    {
      iRow[j] = hessian_iRow[j];
      jCol[j] = hessian_jCol[j];
    }
  }
  return true;
}

bool GMNonlinProblem::get_constraints_linearity(
   Ipopt::Index          m,
   Ipopt::TNLP::LinearityType* const_types
)
{
  assert(m == nbnodes_src + nbnodes_dst);
  for (Ipopt::Index i = 0; i < m; ++i)
    const_types[i] = Ipopt::TNLP::LINEAR;
  return true;
}

void GMNonlinProblem::finalize_solution(
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
  //std::cout << "    METHOD GMNonlinProblem::FINALIZE_SOLUTION()" << std::endl;
  assert(n == nbnodes_src * nbnodes_dst);
  (void) status;
  (void) z_U; (void) z_L;
  (void) m; (void) g;
  (void) lambda; (void) obj_value;
  (void) ip_data; (void) ip_cq;
  // here is where we would store the solution to variables, or write to a file, etc
  // so we could use the solution.
  // For this example, we write the solution to the console
  matching.resize(nbnodes_src, nbnodes_dst);
  for (Ipopt::Index i_src = 0; i_src < nbnodes_src; ++i_src)
  {
    for (Ipopt::Index i_dst = 0; i_dst < nbnodes_dst; ++i_dst)
    {
      Ipopt::Number xi = x[i_src * nbnodes_dst + i_dst];
      if (xi < 0.00001)
        matching(i_src, i_dst) = 0;
      else if (xi > 1 - 0.00001)
        matching(i_src, i_dst) = 1;
      else
      {
        std::cout << "GMNonlinProblem: non-integer value in solution! x["<<i_src<<','<<i_dst<<"] = " << xi << std::endl;
        matching(i_src, i_dst) = static_cast<int> (xi + 0.5); // rounding
      }
    }
  }
}

void GraphMatchingNonlin::run(void)
{
  Ipopt::SmartPtr<Ipopt::TNLP> problemSmartPtr = problem;

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-08);
  app->Options()->SetNumericValue("obj_scaling_factor", -1.0); // MAXIMIZE
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetIntegerValue("print_level", 3);    // verbosity

  Ipopt::ApplicationReturnStatus status;
  status = app->Initialize();
  if( status != Ipopt::Solve_Succeeded )
  {
     std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
     //return (int) status;
  }
  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(problemSmartPtr);
  matching = problem->matching;
  //status = app->OptimizeTNLP(this);
  if( status == Ipopt::Solve_Succeeded )
  {
     std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else
  {
     std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  }
  // clone member problem if you want to save it
  // problemSmartPtr will destroy problem here
}
