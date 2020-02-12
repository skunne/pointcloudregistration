#include <iomanip>  // std::left std::setw()
#include <iostream> // std::cout
#include <vector>
#include <fstream>  // opening and reading files

#include <unistd.h>

#include "cpr_loadfiles.h"
#include "cpr_params.h"
#include "cpr_processedpointcloud.h"
#include "cpr_matrices.h"
#include "cpr_graphmatching_path.h"

#include "test_frankwolfe.h"

int test_printUsage(char const *cmd)
{
  std::cout << "SYNOPSIS" << std::endl << std::endl;
  std::cout << cmd << " <pc0> <pc1>" << std::endl;
  std::cout << "    Use RanSaC to remove mismatched points from the two matched point clouds <pc0> and <pc1>" << std::endl;
  std::cout << "    The two point clouds must be given as csv files with four fields per row: id,x,y,z" << std::endl;
  return 1;
}

int test_loadCSVPointcloud(char const *filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
      std::cout << "Wrong line when reading .csv point cloud:" << std::endl;
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

int main(int argc, char ** argv)
{
  if (argc < 3)
    return test_printUsage(argv[0]);

//  std::vector<char const *> names;
//  std::vector<double> results;

  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_model(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pc_data(new pcl::PointCloud<pcl::PointXYZ>);

  if (test_loadCSVPointcloud(argv[1], pc_model))
    exit(1);
  if (test_loadCSVPointcloud(argv[2], pc_data))
    exit(1);

  std::cout << "Number of points in point cloud: " << pc_model->size() << std::endl;
  if (pc_model->size() != pc_data->size())
  {
    std::cout << "Number of points in second point cloud: " << pc_data->size() << "!! Mismatch!! Aborting..." << std::endl;
    exit(-1);
  }





  return (0);
}
