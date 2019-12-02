#include <algorithm>  // find()
#include "main.h"

int getConnectedComponents(SupervoxelClusters const &vertices,
                          SupervoxelAdjacency const &edges,
                          std::vector<std::vector<KeyT>> &cc_list,
                          std::map<KeyT, std::size_t> &cc_membership)
{
  std::vector<KeyT> nonvisited_vertices;

  for (auto v_itr = vertices.begin(); v_itr != vertices.end(); v_itr++)
    nonvisited_vertices.push_back(v_itr->first);

  std::size_t cc_index = 0;
  while (!nonvisited_vertices.empty())
  {
    std::vector<KeyT> current_cc;
    std::queue<KeyT> vertices_to_be_processed;

    vertices_to_be_processed.push(nonvisited_vertices.back());
    while (!vertices_to_be_processed.empty())
    {
      KeyT v = vertices_to_be_processed.front();
      vertices_to_be_processed.pop();
      nonvisited_vertices.erase(find(nonvisited_vertices.begin(), nonvisited_vertices.end(),v));   // could be optimized
      cc_membership[v] = cc_index;
      current_cc.push_back(v);
      for (auto adjacent_itr = edges.equal_range (v).first; adjacent_itr!=edges.equal_range (v).second; ++adjacent_itr)
        vertices_to_be_processed.push(adjacent_itr->second);
    }
    cc_list.push_back(current_cc);
    cc_index++;
  }
  return (cc_index);
}
