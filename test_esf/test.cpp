#include <pcl/features/esf.h>
#include "cpr_main.h"
#include "cpr_params.h"
#include "cpr_loadfiles.h"

#define HISTOGRAM_SIZE  (640)

int main(int argc, char **argv)
{
  if (argc < 3)
    return printUsage(argv[0]);

  PointCloudT::Ptr cloudalpha;
  PointCloudT::Ptr cloudbeta;
  pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf_calculator;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfalpha;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfbeta;



}
