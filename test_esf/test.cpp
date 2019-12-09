
#include <pcl/features/esf.h>
#include <pcl/visualization/histogram_visualizer.h>
#include "cpr_main.h"
#include "cpr_params.h"     // class Params to read metadata file
#include "cpr_loadfiles.h"  // cpr_loadFile() to input point cloud from vtk/pcd file
//#include "cpr_features.h"   // esfDistance()

#define HISTOGRAM_SIZE  (640)

typedef pcl::ESFEstimation<PointT, pcl::ESFSignature640> ESFMaker;

float esfDistance(float const *a, float const *b)
{
  float d = 0.0;
  for (unsigned int i = 0; i < HISTOGRAM_SIZE; ++i)
  {
    d += (a[i] - b[i]) * (a[i] - b[i]);
    //pcl::console::print_info("bin %d, bin dist %f, accum dist %f\n", i, (a[i] - b[i]) * (a[i] - b[i]), d);
  }
  return d;
}

int main(int argc, char **argv)
{
  if (argc < 2)
    return printUsage(argv[0]);

/*  Params paramsalpha(argv[1]);
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

  //pcl::visualization::PCLHistogramVisualizer visu;
  //visu.addFeatureHistogram(esfalpha, 640, "alpha");
  //visu.spin();
  //visu.addFeatureHistogram(esfbeta, 640, "beta");
  //visu.spin();

  float distance = esfDistance(esfalpha.points[0].histogram, esfbeta.points[0].histogram);

  pcl::console::print_highlight("Distance between the two clouds in ESF space: \n    %f\n", distance);

  */

  pcl::console::print_highlight("\n\n=========\n\n");

  //int nbFiles = argc - 1;

  std::vector<Params *> params;
  std::vector<PointCloudT::Ptr> cloud;
  std::vector<ESFMaker::PointCloudOut::Ptr> esf_out;
  ESFMaker esf_calculator;

  for (int i = 0; i < argc - 1; ++i)
  {
    params.push_back(new Params(argv[i + 1]));
    if (params[i]->error)
      return (params[i]->error);

    cloud.push_back(boost::shared_ptr<PointCloudT>(new PointCloudT));
    int error_loading_file = cpr_loadFile(params[i]->filename.c_str(), params[i]->is_pcd, cloud[i]);
    if (error_loading_file)
      return (error_loading_file);

    esf_calculator.setInputCloud(cloud[i]);
    esf_out.push_back(boost::shared_ptr<ESFMaker::PointCloudOut>(new (ESFMaker::PointCloudOut)));
    esf_calculator.compute(*esf_out[i]);
  }

  Eigen::MatrixXd dist_mat(argc - 1, argc - 1);
  for (int i = 0; i < argc - 1; ++i)
    for (int j = 0; j < argc - 1; ++j)
      dist_mat(i, j) = esfDistance(esf_out[i]->points[0].histogram, esf_out[j]->points[0].histogram);

  std::cout << std::endl << std::endl;
  for (int i = 0; i < argc - 1; ++i)
    std::cout << i << ".  " << params[i]->filename << std::endl;

  std::cout << std::endl << std::endl;
  std::cout << dist_mat;
  std::cout << std::endl << std::endl;


  return (0);
}
