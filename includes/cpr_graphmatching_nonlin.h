#ifndef __DEF_CPR_GraphMatchingNonlin_NONLIN_H__
# define __DEF_CPR_GraphMatchingNonlin_NONLIN_H__

#include <map>
#include <Eigen/Core>
#include <IpTNLP.hpp>

#include "cpr_matrices.h"   // VertexSimilarityMatrix, EdgeSimilarityMatrix
#include "cpr_typedef.h"    // MatrixDouble, MatrixInt

#include "cpr_graphmatching.h"

class GMNonlinProblem : public Ipopt::TNLP, public GraphMatching
{
private:
  Ipopt::Index nbnodes_src;
  Ipopt::Index nbnodes_dst;
  std::vector<Ipopt::Number> x_cached;
  std::vector<Ipopt::Number> xD_cached;
  std::vector<Ipopt::Number> hessian_values;
  std::vector<Ipopt::Index> hessian_iRow;
  std::vector<Ipopt::Index> hessian_jCol;

protected:
  void buildHessian(void);

public:
  GMNonlinProblem(
    MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
    MatrixInt const *g_adj, MatrixInt const *h_adj
  );

  virtual bool get_nlp_info(
    Ipopt::Index&          n,
    Ipopt::Index&          m,
    Ipopt::Index&          nnz_jac_g,
    Ipopt::Index&          nnz_h_lag,
    Ipopt::TNLP::IndexStyleEnum& index_style
  );

  virtual bool get_bounds_info(
    Ipopt::Index   n,
    Ipopt::Number* x_l,
    Ipopt::Number* x_u,
    Ipopt::Index   m,
    Ipopt::Number* g_l,
    Ipopt::Number* g_u
  );

  virtual bool get_starting_point(
    Ipopt::Index   n,
    bool    init_x,
    Ipopt::Number* x,
    bool    init_z,
    Ipopt::Number* z_L,
    Ipopt::Number* z_U,
    Ipopt::Index   m,
    bool    init_lambda,
    Ipopt::Number* lambda
  );

  virtual bool eval_f(
    Ipopt::Index         n,
    const Ipopt::Number* x,
    bool          new_x,
    Ipopt::Number&       obj_value
  );

  virtual bool eval_grad_f(
    Ipopt::Index         n,
    const Ipopt::Number* x,
    bool          new_x,
    Ipopt::Number*       grad_f
  );

  virtual bool eval_g(
    Ipopt::Index         n,
    const Ipopt::Number* x,
    bool          new_x,
    Ipopt::Index         m,
    Ipopt::Number*       g
  );

  virtual bool eval_jac_g(
    Ipopt::Index         n,
    const Ipopt::Number* x,
    bool          new_x,
    Ipopt::Index         m,
    Ipopt::Index         nele_jac,
    Ipopt::Index*        iRow,
    Ipopt::Index*        jCol,
    Ipopt::Number*       values
  );

  virtual bool eval_h(
     Ipopt::Index         n,
     const Ipopt::Number* x,
     bool          new_x,
     Ipopt::Number        obj_factor,
     Ipopt::Index         m,
     const Ipopt::Number* lambda,
     bool          new_lambda,
     Ipopt::Index         nele_hess,
     Ipopt::Index*        iRow,
     Ipopt::Index*        jCol,
     Ipopt::Number*       values
  );

  virtual bool get_constraints_linearity(
     Ipopt::Index          m,
     Ipopt::TNLP::LinearityType* const_types
  );

  virtual void finalize_solution(
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
  );
};

class GraphMatchingNonlin : public GraphMatching
{
private:
  GMNonlinProblem *problem;

public:
  GraphMatchingNonlin(
    MatrixDouble const *vsim, EdgeSimilarityMatrix const *esim,
    MatrixInt const *g_adj, MatrixInt const *h_adj
  );

  void run(void);
};



#endif /* __DEF_CPR_GraphMatchingNonlin_NONLIN_H__ */
