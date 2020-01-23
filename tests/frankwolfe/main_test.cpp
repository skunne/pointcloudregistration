#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>

// test_house.cpp
double test_two_house_graphs();

// test_twonodes.cpp
double test_two_twonodes_graphs();

// test_differentnumbernodes.cpp
double test_5nodes_with_6nodes();

// test_multipleoptimalsolutions.cpp
double test_multiple_optimal_solutions();

int main(void)
{
  std::vector<char const *> names = { "two nodes", "houses", "5 vs 6", "multiple optimal" };
  std::vector<double> diff_with_human(4);

  diff_with_human[0] = test_two_twonodes_graphs();

  diff_with_human[1] = test_two_house_graphs();

  diff_with_human[2] = test_5nodes_with_6nodes();

  diff_with_human[3] = test_multiple_optimal_solutions();

  std::cout << std::endl << std::endl << "Tests passed:" << std::endl;
  for (std::size_t i = 0; i < diff_with_human.size(); ++i)
    std::cout << std::left << std::setw(16) << names[i] << "  " << diff_with_human[i] << std::endl;

  return (0);
}
