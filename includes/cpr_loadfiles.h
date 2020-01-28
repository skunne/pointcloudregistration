
#ifndef __DEF_LOADFILES_H__
# define __DEF_LOADFILES_H__

#include "cpr_params.h"
#include "cpr_main.h"

/*
** cpr_loadfiles.cpp
*/
int cpr_loadFile(char const *filename, Params *params, PointCloudT::Ptr cloud);
//int cpr_loafFile(Params const &params, PointCloudT::Ptr cloud);
int errorLoadingFile(char const *type, char const *name);
int loadVTKFile(char const *filename, PointCloudT::Ptr cloud);
int loadPCDFile(char const *filename, PointCloudT::Ptr cloud);
int loadPLYFile(char const *filename, PointCloudT::Ptr cloud);

#endif /* __DEF_LOADFILES_H__ */
