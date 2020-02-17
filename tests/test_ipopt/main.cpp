#include <iostream>

#include <IpIpoptApplication.hpp>

#include "nonlinprogram.hpp"



int main(void)
{
  Ipopt::SmartPtr<Ipopt::TNLP> problem = new NonLinProgram();

  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();
  app->Options()->SetNumericValue("tol", 1e-7);
  app->Options()->SetStringValue("mu_strategy", "adaptive");
  app->Options()->SetStringValue("output_file", "ipopt.out");
  app->Options()->SetIntegerValue("print_level", 0);    // verbosity

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
  return (int) status;
}
