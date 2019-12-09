
#include <pcl/features/esf.h>
#include <pcl/visualization/histogram_visualizer.h>
#include "cpr_main.h"
#include "cpr_params.h"     // class Params to read metadata file
#include "cpr_loadfiles.h"  // cpr_loadFile() to input point cloud from vtk/pcd file
//#include "cpr_features.h"   // esfDistance()

#define HISTOGRAM_SIZE  (640)

float esfDistance(float const *a, float const *b)
{
  float d = 0;
  for (unsigned int i = 0; i < HISTOGRAM_SIZE; ++i)
    d += (a[i] - b[i]) * (a[i] - b[i]);
  return d;
}

int main(int argc, char **argv)
{
  if (argc < 3)
    return printUsage(argv[0]);

  Params paramsalpha(argv[1]);
  Params paramsbeta(argv[2]);
  if (paramsalpha.error || paramsbeta.error)
    return (1);

  PointCloudT::Ptr cloudalpha(new PointCloudT);
  PointCloudT::Ptr cloudbeta(new PointCloudT);
  pcl::ESFEstimation<PointT, pcl::ESFSignature640> esf_calculator;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfalpha;
  typename pcl::ESFEstimation<PointT, pcl::ESFSignature640>::PointCloudOut esfbeta;

  int error_loading_file = cpr_loadFile(paramsalpha.filename.c_str(), paramsalpha.is_pcd, cloudalpha);
  error_loading_file = error_loading_file || cpr_loadFile(paramsbeta.filename.c_str(), paramsbeta.is_pcd, cloudbeta);
  if (error_loading_file)
    return (error_loading_file);

  esf_calculator.setInputCloud(cloudalpha);
  esf_calculator.compute(esfalpha);

  esf_calculator.setInputCloud(cloudbeta);
  esf_calculator.compute(esfbeta);

  pcl::visualization::PCLHistogramVisualizer visu;
  visu.addFeatureHistogram(esfalpha, 640, "alpha");
  visu.spin();
  visu.addFeatureHistogram(esfbeta, 640, "beta");
  visu.spin();

  int distance = esfDistance(esfalpha.points[0].histogram, esfbeta.points[0].histogram);

  pcl::console::print_highlight("Distance between the two clouds in ESF space: \n    %f\n", distance);

  return (0);
}
