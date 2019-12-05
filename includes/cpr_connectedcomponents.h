
#ifndef __DEF_CONNECTEDCOMPONENTS_H__
# define __DEF_CONNECTEDCOMPONENTS_H__
#include "cpr_main.h"


/*
** connectedcomponents.cpp
*/

int getConnectedComponents(SupervoxelClusters const &vertices,
                          SupervoxelAdjacency const &edges,
                          std::vector<std::vector<KeyT>> &cc_list);
                          //std::map<KeyT, std::size_t> &cc_membership);

/*
** add one vertex at center of each cc
** add edges from this center to each vertex in the cc
** add edges from all centers to all centers
*/
void makeGraphConnected(SupervoxelClusters &vertices,
                        SupervoxelAdjacency &edges,
                        std::vector<std::vector<KeyT>> &cc_list);
                        //std::map<KeyT, std::size_t> &cc_membership);

#endif /* __DEF_CONNECTEDCOMPONENTS_H__ */
