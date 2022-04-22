
#ifndef __DEF_CPR_DEBUG_SUPERVOXEL_H__
# define __DEF_CPR_DEBUG_SUPERVOXEL_H__
#include "cpr_main.h"

namespace cprdbg
{
  namespace supervoxel
  {
    void print_centroids(SupervoxelClusters const &supervoxel_clusters, int verbosity);
    void print_pointcloud(PointCloudT::Ptr const cloud, int verbosity);
  }
}


#endif /* __DEF_CPR_DEBUG_FRANKWOLFE_H__ */
