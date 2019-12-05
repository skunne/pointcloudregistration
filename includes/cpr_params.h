
#ifndef __DEF_PARAMS_H__
# define __DEF_PARAMS_H__
#include "cpr_main.h"

struct Params
{
  bool  is_pcd;
  //bool  disable_transform;
  float voxel_resolution;
  bool  voxel_res_specified;
  float seed_resolution;
  bool  seed_res_specified;
  float color_importance;
  float spatial_importance;
  float normal_importance;
};

int setParams(int argc, char **argv, struct Params *params);

#endif /* __DEF_PARAMS_H__ */
