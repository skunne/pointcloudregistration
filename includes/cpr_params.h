
#ifndef __DEF_CPR_PARAMS_H__
# define __DEF_CPR_PARAMS_H__
#include "cpr_main.h"

class Params
{
public:
  int error;
  std::string filename;
  bool  is_pcd;
  bool  is_vtk;
  //bool  disable_transform;
  float voxel_resolution;
  bool  voxel_res_specified;
  float seed_resolution;
  bool  seed_res_specified;
  float color_importance;
  float spatial_importance;
  float normal_importance;

public:
  Params(char const *metadata_filename);
  Params(Params const &params) = default;
};

bool endsWith(const std::string& a, const std::string& b);

int printUsage(char const *command);

int getParams(int argc, char const *const *argv, struct Params *params);

#endif /* __DEF_CPR_PARAMS_H__ */
