
#include <iostream>   // std::cout
#include <iomanip>    // std::fixed, std::setprecision to print doubles/floats

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

  pcl::console::print_highlight("\n\n=========\n\n");

  //int nbFiles = argc - 1;

  std::vector<Params *> params;
  std::vector<PointCloudT::Ptr> cloud;
  std::vector<ESFMaker::PointCloudOut::Ptr> esf_out;
  ESFMaker esf_calculator;

  // compute ESF descriptor for each point cloud (one point cloud per file)
  for (int i = 0; i < argc - 1; ++i)
  {
    params.push_back(new Params(argv[i + 1]));
    if (params[i]->error)
      return (params[i]->error);

    cloud.push_back(boost::shared_ptr<PointCloudT>(new PointCloudT));
    int error_loading_file = cpr_loadFile(params[i]->filename.c_str(), params[i], cloud[i]);
    if (error_loading_file)
      return (error_loading_file);

    esf_calculator.setInputCloud(cloud[i]);
    esf_out.push_back(boost::shared_ptr<ESFMaker::PointCloudOut>(new (ESFMaker::PointCloudOut)));
    esf_calculator.compute(*esf_out[i]);
  }

  // compute matrix of distances
  // dist_mat(i, j) = ESF distance between point cloud i and pointcloud j
  MatrixDouble dist_mat(argc - 1, argc - 1);
  for (int i = 0; i < argc - 1; ++i)
    for (int j = 0; j < argc - 1; ++j)
      dist_mat(i, j) = esfDistance(esf_out[i]->points[0].histogram, esf_out[j]->points[0].histogram);

  // print numbered list of point clouds
  std::cout << std::endl << "List of point clouds:" << std::endl;
  for (int i = 0; i < argc - 1; ++i)
    std::cout << i << ".  " << argv[i + 1] << std::endl;

  // print distance matrix between point clouds
  std::cout << std::endl << "ESF distance matrix:" << std::endl;
  std::cout << std::scientific << std::setprecision(0);
  std::cout << dist_mat;
  std::cout << std::endl << std::endl;


  return (0);
}
