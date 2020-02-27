#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "test_icpransac.h"

void get_point_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dest)
{
  // Fill in the CloudIn data
  cloud_source->width    = 5;
  cloud_source->height   = 1;
  cloud_source->is_dense = false;
  cloud_source->points.resize (cloud_source->width * cloud_source->height);
  for (std::size_t i = 0; i < cloud_source->points.size (); ++i)
  {
    cloud_source->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_source->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud_source->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }
  std::cout << "Saved " << cloud_source->points.size () << " data points to input:"
      << std::endl;
  for (std::size_t i = 0; i < cloud_source->points.size (); ++i) std::cout << "    " <<
      cloud_source->points[i].x << " " << cloud_source->points[i].y << " " <<
      cloud_source->points[i].z << std::endl;
  *cloud_dest = *cloud_source;
  std::cout << "size:" << cloud_dest->points.size() << std::endl;
  for (std::size_t i = 0; i < cloud_source->points.size (); ++i)
    cloud_dest->points[i].x = cloud_source->points[i].x + 0.7f;
  std::cout << "Transformed " << cloud_source->points.size () << " data points:"
      << std::endl;
  for (std::size_t i = 0; i < cloud_dest->points.size (); ++i)
    std::cout << "    " << cloud_dest->points[i].x << " " <<
      cloud_dest->points[i].y << " " << cloud_dest->points[i].z << std::endl;
}

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_dest (new pcl::PointCloud<pcl::PointXYZ>);

  get_point_clouds(cloud_source, cloud_dest);

  Eigen::Matrix4f icp_transform;
  double icp_score = icpransac(cloud_source, cloud_dest, icp_transform);

  std::cout << "ICP/Ransac score: " << icp_score << std::endl;
  std::cout << "ICP/Ransac transform:" << std::endl << icp_transform << std::endl;



 return (0);
}
