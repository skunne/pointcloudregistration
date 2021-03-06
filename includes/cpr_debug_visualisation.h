
#ifndef __DEF_CPR_DEBUG_VISUALISATION_H__
# define __DEF_CPR_DEBUG_VISUALISATION_H__
#include "cpr_main.h"

namespace cprdbg
{
  namespace visualisation
  {
    int const verbosity = 3;

    void print_point_index_and_colour(
      KeyT vertex, int index,
      double r, double g, double b, int verbosity);

    void print_both_sets_of_sample_points(
      std::vector<KeyT> const &sample_g,
      std::vector<KeyT> const &sample_h);
  }
}


#endif /* __DEF_CPR_DEBUG_VISUALISATION_H__ */
