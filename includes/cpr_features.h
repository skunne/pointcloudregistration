
#ifndef __DEF_FEATURES_H__
# define __DEF_FEATURES_H__

#include "cpr_main.h"


/*
** features.cpp
*/

void calculateAnglesAndLength(PointT const &p1, PointT const &p2, double &angle_x, double &angle_y, double &angle_z, double &length);

void calculateESFDescriptors(SupervoxelClusters const &sv_clusters, ESFDescriptors &esf_descriptors);

void calculateEdgesDescriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, EdgeDescriptors &edge_descriptors);

void calculateDescriptors(SupervoxelClusters const &sv_clusters, SupervoxelAdjacency const &sv_adjacency, ESFDescriptors &esf_descriptors, EdgeDescriptors &edge_descriptors);

void writeDescriptorsToCSV(char const *name, ESFDescriptors esf_descriptors, EdgeDescriptors edge_descriptors);



#endif /* __DEF_FEATURES_H__ */
