
#ifndef __DEF_CPR_PARAMS_H__
# define __DEF_CPR_PARAMS_H__
#include "cpr_main.h"

class Params
{
public:
  int error;

  // input point cloud
  std::string filename;
  bool  is_pcd;
  bool  is_vtk;
  bool  is_ply;

  // supervoxel clustering
  float voxel_resolution;
  bool  voxel_res_specified;
  float seed_resolution;
  bool  seed_res_specified;
  float color_importance;
  float spatial_importance;
  float normal_importance;

  // output adjacency matrix
  std::string adjacency_filename;


public:
  Params(char const *metadata_filename);
  Params(Params const &params) = default;

  float get_seed_resolution(void) const;
  float get_voxel_resolution(void) const;
};

bool endsWith(const std::string& a, const std::string& b);

int printUsage(char const *command);

int setParams(int argc, char const *const *argv, struct Params *params);

#endif /* __DEF_CPR_PARAMS_H__ */
