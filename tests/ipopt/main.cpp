#include <iostream>
#include <map>
#include <Eigen/Core>

#include <IpIpoptApplication.hpp>

#include "nonlinprogram.hpp"

class TwoGraphs
{
public:
  Ipopt::Index nbnodes_src;
  Ipopt::Index nbnodes_dst;
  Eigen::MatrixXd vertex_similarity;
  std::map<std::pair<Ipopt::Index, Ipopt::Index>, Ipopt::Index> sourceEdgeIndex;
  std::map<std::pair<Ipopt::Index, Ipopt::Index>, Ipopt::Index> destEdgeIndex;
  Eigen::MatrixXd edge_similarity;

public:
  TwoGraphs();
};

TwoGraphs::TwoGraphs()
  : nbnodes_src(6), nbnodes_dst(6),
  vertex_similarity(6,6),
  edge_similarity(2*7,2*7)
{
  vertex_similarity.setOnes();
  // vertex_similarity <<
  //   1.0, 0.5, 0.5, 0.5, 0.5, 0.5,
  //   0.5, 1.0, 0.6, 0.6, 0.9, 0.5,
  //   0.5, 0.6, 1.0, 0.9, 0.6, 0.5,
  //   0.5, 0.5, 0.9, 0.5, 0.5, 0.5,
  //   0.5, 0.6, 0.9, 1.0, 0.6, 0.5,
  //   0.5, 0.5, 0.6, 0.6, 1.0, 0.5;

  sourceEdgeIndex[std::make_pair(0,1)] = 0;
  sourceEdgeIndex[std::make_pair(1,2)] = 2;
  sourceEdgeIndex[std::make_pair(2,4)] = 4;
  sourceEdgeIndex[std::make_pair(4,5)] = 6;
  sourceEdgeIndex[std::make_pair(5,0)] = 8;
  sourceEdgeIndex[std::make_pair(5,1)] = 10;
  sourceEdgeIndex[std::make_pair(2,3)] = 12;

  sourceEdgeIndex[std::make_pair(1,0)] = 1;
  sourceEdgeIndex[std::make_pair(2,1)] = 3;
  sourceEdgeIndex[std::make_pair(4,2)] = 5;
  sourceEdgeIndex[std::make_pair(5,4)] = 7;
  sourceEdgeIndex[std::make_pair(0,5)] = 9;
  sourceEdgeIndex[std::make_pair(1,5)] = 11;
  sourceEdgeIndex[std::make_pair(3,2)] = 13;

  destEdgeIndex[std::make_pair(0,1)] = 0;
  destEdgeIndex[std::make_pair(1,2)] = 2;
  destEdgeIndex[std::make_pair(2,3)] = 4;
  destEdgeIndex[std::make_pair(3,4)] = 6;
  destEdgeIndex[std::make_pair(4,0)] = 8;
  destEdgeIndex[std::make_pair(4,5)] = 10;
  destEdgeIndex[std::make_pair(4,1)] = 12;

  destEdgeIndex[std::make_pair(1,0)] = 1;
  destEdgeIndex[std::make_pair(2,1)] = 3;
  destEdgeIndex[std::make_pair(3,2)] = 5;
  destEdgeIndex[std::make_pair(4,3)] = 7;
  destEdgeIndex[std::make_pair(0,4)] = 9;
  destEdgeIndex[std::make_pair(5,4)] = 11;
  destEdgeIndex[std::make_pair(1,4)] = 13;

  edge_similarity.setOnes();
  // edge_similarity <<
  //   1.0,1.0,0.1,0.1,0.2,0.2,0.3,0.3,0.9,0.9,1.0,1.0,0.1,0.1,
  //   1.0,1.0,0.1,0.1,0.2,0.2,0.3,0.3,0.9,0.9,1.0,1.0,0.1,0.1,
  //   0.1,0.1,1.0,1.0,0.0,0.0,1.0,1.0,0.2,0.2,0.9,0.9,0.1,0.1,
  //   0.1,0.1,1.0,1.0,0.0,0.0,1.0,1.0,0.2,0.2,0.9,0.9,0.1,0.1,
  //   0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.1,0.1,1.0,1.0,1.0,1.0,
  //   0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.1,0.1,1.0,1.0,1.0,1.0,
  //   0.2,0.2,1.0,1.0,0.2,0.2,1.0,1.0,0.2,0.2,1.0,1.0,0.2,0.2,
  //   0.2,0.2,1.0,1.0,0.2,0.2,1.0,1.0,0.2,0.2,1.0,1.0,0.2,0.2,
  //   0.7,0.7,1.0,1.0,0.3,0.3,0.2,0.2,1.0,1.0,1.0,1.0,0.1,0.1,
  //   0.7,0.7,1.0,1.0,0.3,0.3,0.2,0.2,1.0,1.0,1.0,1.0,0.1,0.1,
  //   0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.1,0.1,0.4,0.4,0.1,0.1,
  //   0.1,0.1,0.1,0.1,1.0,1.0,0.1,0.1,0.1,0.1,0.4,0.4,0.1,0.1,
  //   0.6,0.6,0.6,0.6,0.8,0.8,0.6,0.6,0.6,0.6,1.0,1.0,0.7,0.7,
  //   0.6,0.6,0.6,0.6,0.8,0.8,0.6,0.6,0.6,0.6,1.0,1.0,0.7,0.7;
}

int main(void)
{
  //std::cout << "    FUNCTION MAIN()............0" << std::endl;
  TwoGraphs two_graphs;
  //std::cout << "    FUNCTION MAIN()...........10" << std::endl;
  Ipopt::SmartPtr<Ipopt::TNLP> problem = new GraphMatching(
    &two_graphs.vertex_similarity,
    &two_graphs.edge_similarity,
    &two_graphs.sourceEdgeIndex,
    &two_graphs.destEdgeIndex
  );
  //std::cout << "    FUNCTION MAIN()...........20" << std::endl;
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-15);
  app->Options()->SetNumericValue("obj_scaling_factor", -1.0); // MAXIMIZE
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetIntegerValue("print_level", 3);    // verbosity

  //std::cout << "    FUNCTION MAIN()...........30" << std::endl;

  Ipopt::ApplicationReturnStatus status;
  status = app->Initialize();
  if( status != Ipopt::Solve_Succeeded )
  {
     std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
     return (int) status;
  }
  // Ask Ipopt to solve the problem
  status = app->OptimizeTNLP(problem);
  if( status == Ipopt::Solve_Succeeded )
  {
     std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
  }
  else
  {
     std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
  }
  // As the Ipopt::SmartPtrs go out of scope, the reference count
  // will be decremented and the objects will automatically
  // be deleted.

  //std::cout << "    FUNCTION MAIN().........1000" << std::endl;

  return (int) status;
}
