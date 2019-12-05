#include <algorithm>  // find()
#include "cpr_main.h"
#include "cpr_connectedcomponents.h"

int getConnectedComponents(SupervoxelClusters const &vertices,
                          SupervoxelAdjacency const &edges,
                          std::vector<std::vector<KeyT>> &cc_list)
                          //std::map<KeyT, std::size_t> &cc_membership)
{
  std::vector<KeyT> nonvisited_vertices;  // std::vector is not a very good type for this

  //pcl::console::print_info("getCC(): initialize nonvisited_vertices\n");
  for (auto v_itr = vertices.begin(); v_itr != vertices.end(); v_itr++)
    nonvisited_vertices.push_back(v_itr->first);

  //pcl::console::print_info("getCC(): main loop\n");
  std::size_t cc_index = 0;
  while (!nonvisited_vertices.empty())
  {
    std::vector<KeyT> current_cc;
    std::queue<KeyT> vertices_to_be_processed;

    vertices_to_be_processed.push(nonvisited_vertices.back());

    //pcl::console::print_info("  getCC(): enter inner loop\n");
    while (!(vertices_to_be_processed.empty()))
    {
      KeyT v = vertices_to_be_processed.front();
      vertices_to_be_processed.pop();
      //pcl::console::print_info("    getCC() inner loop: popped one\n");
      auto x = find(nonvisited_vertices.begin(), nonvisited_vertices.end(),v);
      if (x == nonvisited_vertices.end())
      {
        ;
        //pcl::console::print_info("    getCC() inner loop: element not found\n");
      }
      else
      {
        //pcl::console::print_info("    getCC() inner loop: found to erase\n");
        nonvisited_vertices.erase(x);   // could be optimized
        //pcl::console::print_info("    getCC() inner loop: erased\n");
        //cc_membership[v] = cc_index;
        current_cc.push_back(v);
        //pcl::console::print_info("    getCC() inner loop: added v to current_cc\n");
        for (auto adjacent_itr = edges.equal_range (v).first; adjacent_itr!=edges.equal_range (v).second; ++adjacent_itr)
          vertices_to_be_processed.push(adjacent_itr->second);
        //pcl::console::print_info("    getCC() inner loop: added neighbours to queue\n");
      }
    }
    //pcl::console::print_info("  getCC(): exit inner loop\n");
    cc_list.push_back(current_cc);
    cc_index++;
  }
  //pcl::console::print_info("getCC(): exit main loop\n");
  return (cc_index);
}

void getCentreOfMass(std::vector<KeyT> const pointLabels, SupervoxelClusters const &vertices, PointT &c)
{
  //PointT c;     // centre of mass
  c.x = 0;
  c.y = 0;
  c.z = 0;
  std::size_t nbPoints = 0;
  for (std::vector<KeyT>::const_iterator lbl_itr = pointLabels.cbegin();
      lbl_itr != pointLabels.cend();
      lbl_itr++)
  {
    PointT p;     // supervoxel with label (*lbl_itr)
    vertices.at(*lbl_itr)->getCentroidPoint(p);
    c.x += p.x;
    c.y += p.y;
    c.z += p.z;
    nbPoints++;
  }
  c.x = c.x / nbPoints;
  c.y = c.y / nbPoints;
  c.z = c.z / nbPoints;
  //return c;
}

/*
** Create a new point at the center of each connected component
** This new point has an edge to each point in the connected component
** Plus an edge to each other connected component center
*/
void makeGraphConnected(SupervoxelClusters &vertices,
                        SupervoxelAdjacency &edges,
                        std::vector<std::vector<KeyT>> &cc_list)
                        //std::map<KeyT, std::size_t> &cc_membership)
{
  // get label names for insertion into SupervoxelClusters
  KeyT nextFreeLabel = 0;
  for (auto v_itr = vertices.begin(); v_itr != vertices.end(); v_itr++)
    nextFreeLabel = (nextFreeLabel > v_itr->first) ? nextFreeLabel : v_itr->first;
  nextFreeLabel++;

  // add center point for each cc and add edge from it to each point in cc
  KeyT clusterIndex = 0;
  for (auto cc_itr = cc_list.begin(); cc_itr != cc_list.end(); cc_itr++)
  {
    pcl::Supervoxel<PointT>::Ptr cc_centre(new pcl::Supervoxel<PointT>);
    //cc_centre->centroid_ = getCentreOfMass(*cc_itr, vertices);
    getCentreOfMass(*cc_itr, vertices, cc_centre->centroid_);
    vertices[nextFreeLabel + clusterIndex] = cc_centre;
    for (auto lbl_itr = cc_itr->begin(); lbl_itr != cc_itr->end(); lbl_itr++)
    {
      edges.insert(std::make_pair(nextFreeLabel + clusterIndex, *lbl_itr));
      edges.insert(std::make_pair(*lbl_itr, nextFreeLabel + clusterIndex));
    }
    clusterIndex++;

    // add edges between all cc centers so the graph is connected
    for (KeyT i = 1; i < clusterIndex; i++)
      for (KeyT j = 0; j < i; j++)
      {
        edges.insert(std::make_pair(nextFreeLabel + i, nextFreeLabel + j));
        edges.insert(std::make_pair(nextFreeLabel + j, nextFreeLabel + i));
      }
  }
}
