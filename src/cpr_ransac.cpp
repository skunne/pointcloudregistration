// #include <pcl/filters/extract_indices.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
// #include <pcl/sample_consensus/sac_model_plane.h>   // TODO remove this header
// #include <pcl/sample_consensus/sac_model_sphere.h>  // TODO remove this header
#include <pcl/sample_consensus/sac_model_registration.h>
// #include <pcl/visualization/pcl_visualizer.h>

#include "cpr_ransac.h"

void performRansacKeepInliers(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_data,
      pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_model,
      double threshold, //pcl::PointCloud<pcl::PointXYZ>::Ptr &inlier_points)
      std::vector<int> &inlier_indices)
{
  //std::vector<int> inlier_indices;
  pcl::SampleConsensusModelRegistration<pcl::PointXYZ>::Ptr
    model(new pcl::SampleConsensusModelRegistration<pcl::PointXYZ>(cloud_data));
  model->setInputTarget(cloud_model);
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
  ransac.setDistanceThreshold(threshold);
  ransac.computeModel();
  ransac.getInliers(inlier_indices);
  //pcl::copyPointCloud (*cloud_data, inlier_indices, *inlier_points);
}
