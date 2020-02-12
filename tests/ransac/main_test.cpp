#include <vector>   // vector to store inlier point indices
#include <fstream>  // std::ifstream for opening and reading files
#include <sstream>  // std::istringstream for parsing lines
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>    // pcl::PointXYZ
#include <pcl/sample_consensus/ransac.h>  // pcl::RandomSampleConsensus
#include <pcl/sample_consensus/sac_model_registration.h>  // pcl::SampleConsensusModelRegistration

int test_printUsage(char const *cmd)
{
  std::cout << "SYNOPSIS" << std::endl << std::endl;
  std::cout << cmd << " <pc0> <pc1> <out0> <out1>" << std::endl;
  std::cout << "    Use RanSaC to remove mismatched points from the two matched point clouds <pc0> and <pc1>" << std::endl;
  std::cout << "    The two point clouds must be given as csv files with four fields per row: id,x,y,z" << std::endl;
  std::cout << "    The resulting trimmed point clouds are written with the same format on files <out0> and <out1>, overwriting them if they exist" << std::endl;
  return 1;
}

int test_readCSVPointcloud(char const *filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  std::cout << "Loading point cloud from csv file " << filename << std::endl;
  std::ifstream file(filename);
  std::string line;
  std::getline(file, line);   // header: "id,dimension_1,dimension_2,dimension_3"
  while (std::getline(file, line))
  {
    std::istringstream iss(line);
    int id;
    float x,y,z;
    char separator;
    if (!(iss >> id >> separator >> x >> separator >> y >> separator >> z))
    {
      std::cout << "Error parsing line from .csv point cloud:" << std::endl;
      std::cout << "    " << line << std::endl;
      return 1;
    }
    cloud->push_back(pcl::PointXYZ(x,y,z));
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  return 0;
}

void test_writeCSVPointcloud(char const *filename, pcl::PointCloud<pcl::PointXYZ>::Ptr const cloud)
{
  std::ofstream file(filename);
  file << "id,dimension_1,dimension_2,dimension_3" << '\n';
  std::size_t i = 0;
  for (auto p : cloud->points)
  {
    file << i << ',' << p.x << ',' << p.y << ',' << p.z << '\n';
    ++i;
    //note that it is a bit stupid to discard the indices that were read in the input csv file
    //we might want to keep the same indices, so it is apparent which points were outliers
  }
}

int main(int argc, char ** argv)
{
  if (argc < 5)
    return test_printUsage(argv[0]);

  double threshold = 0.01;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data(new pcl::PointCloud<pcl::PointXYZ>);

  if (test_readCSVPointcloud(argv[1], cloud_model))
    exit(1);
  if (test_readCSVPointcloud(argv[2], cloud_data))
    exit(1);

  std::cout << "Number of points in point cloud: " << cloud_model->size() << std::endl;
  if (cloud_model->size() != cloud_data->size())
  {
    std::cout << "Number of points in second point cloud: " << cloud_data->size() << "!! Mismatch!! Aborting..." << std::endl;
    exit(-1);
  }

  pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr
    model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloud_data));
  model->setInputTarget(cloud_model);

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model);
  ransac.setDistanceThreshold(threshold);
  ransac.computeModel();
  std::vector<int> inliers;
  ransac.getInliers(inliers);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_model_inliers(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_data_inliers(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::copyPointCloud (*cloud_data, inliers, *cloud_data_inliers);
  pcl::copyPointCloud (*cloud_model, inliers, *cloud_model_inliers);

  test_writeCSVPointcloud(argv[3], cloud_model_inliers);
  test_writeCSVPointcloud(argv[4], cloud_data_inliers);

  return (0);
}
