
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>

#include "test_icpransac.h"

double icpransac(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_dest, Eigen::Matrix4f &transform)
{
  pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
  icp.setInputSource(cloud_source);
  icp.setInputTarget(cloud_dest);

  /* ICP PARAMETERS*/
  //icp.setMaximumIterations(nr_iterations);  // maximum nb iterations of icp
  //icp.setTransformationEpsilon(epsilon);    // halt if transformation change by less than epsilon
  //icp.setEuclideanFitnessEpsilon(distance);  // say a solution was found if sumofsquared errors < distance

  /* RANSAC PARAMETERS */
  //icp.setMaxCorrespondenceDistance (distance);
  //icp.setRANSACOutlierRejectionThreshold (distance);

  pcl::PointCloud<pcl::PointXYZRGBA> Final;
  icp.align(Final);
  transform = icp.getFinalTransformation();
  return (icp.getFitnessScore());
}
