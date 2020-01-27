#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>

#include "test_frankwolfe.h"

int main(int argc, char ** argv)
{
  std::vector<char const *> names;// = { "two nodes", "houses", "5 vs 6", "multiple optimal", "pointclouds" };
  std::vector<double> diff_with_human;//(5);

  names.push_back("two nodes");
  diff_with_human.push_back(test_two_twonodes_graphs());

  names.push_back("houses");
  diff_with_human.push_back(test_two_house_graphs());

  names.push_back("5 vs 6");
  diff_with_human.push_back(test_5nodes_with_6nodes());

  names.push_back("multiple optimal");
  diff_with_human.push_back(test_multiple_optimal_solutions());

  //int ac = 3;
  //char const *av[3] = {"./tests/frankwolfe/test_frankwolfe", "metadata/big1.meta", "metadata/big3.meta"};
  names.push_back("pointclouds");
  diff_with_human.push_back(test_with_pointclouds(argc, argv));

  std::cout << std::endl << std::endl << "Tests passed:" << std::endl;
  for (std::size_t i = 0; i < diff_with_human.size(); ++i)
    std::cout << std::left << std::setw(16) << names[i] << "  " << diff_with_human[i] << std::endl;

  return (0);
}
