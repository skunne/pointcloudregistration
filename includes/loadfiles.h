
#ifndef __DEF_LOADFILES_H__
# define __DEF_LOADFILES_H__

#include "main.h"

int loadFile(char const *filename, bool is_pcd, PointCloudT::Ptr cloud);
int loadVTKFile(char const *filename, PointCloudT::Ptr cloud);
int loadPCDFile(char const *filename, PointCloudT::Ptr cloud);

#endif /* __DEF_LOADFILES_H__ */
