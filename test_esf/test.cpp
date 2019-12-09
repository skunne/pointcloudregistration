
#include <pcl/features/esf.h>
#include "cpr_main.h"
#include "cpr_params.h"
#include "cpr_loadfiles.h"

#define HISTOGRAM_SIZE  (640)

int main(int argc, char **argv)
{
  if (argc < 3)
    return printUsage(argv[0]);

  Params paramsalpha(argv[1]);
  Params paramsbeta(argv[2]);
  if (paramsalpha.error || paramsbeta.error)
    return (1);

  pcl::console::print_info("Loaded: meta files.\n");

  PointCloudT::Ptr cloudalpha(new PointCloudT);
  PointCloudT::Ptr cloudbeta(new PointCloudT);
  pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf_calculator;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfalpha;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfbeta;

  int error_loading_file = cpr_loadFile(paramsalpha.filename.c_str(), paramsalpha.is_pcd, cloudalpha);
  error_loading_file = error_loading_file || cpr_loadFile(paramsbeta.filename.c_str(), paramsbeta.is_pcd, cloudbeta);
  if (error_loading_file)
    return (1);

  pcl::console::print_info("Loaded: point cloud files.\n");

  return (0);
}
