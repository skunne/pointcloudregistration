

#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_visualisation.h"

void visualisation(ProcessedPointCloud &source, ProcessedPointCloud &dest)
//void visualisation(pcl::SupervoxelClustering<PointT> const &super, SupervoxelClusters &supervoxel_clusters, SupervoxelAdjacency const &supervoxel_adjacency)
{
  pcl::console::print_highlight ("Initialising visualisation\n");

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  source.addToViewer(viewer);
  dest.addToViewer(viewer);
  //visualiseOne(pcl::visualization::PCLVisualizer::Ptr, source.super, source.supervoxel_clusters, source.supervoxel_adjacency, colour1);
  //visualiseOne(pcl::visualization::PCLVisualizer::Ptr, source, colour2);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }
}

void ProcessedPointCloud::addToViewer(pcl::visualization::PCLVisualizer::Ptr viewer)
{
    //pcl::console::print_highlight ("PRINT 10\n");
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  //pcl::console::print_highlight ("PRINT 12\n");
  viewer->addPointCloud (voxel_centroid_cloud, "voxel centroids");
  //pcl::console::print_highlight ("PRINT 14\n");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, "voxel centroids");
  //pcl::console::print_highlight ("PRINT 16\n");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, "voxel centroids");

  //pcl::console::print_highlight ("PRINT 20\n");
  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  viewer->addPointCloud (labeled_voxel_cloud, "labeled voxels");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, "labeled voxels");

  //pcl::console::print_highlight ("PRINT 30\n");
  PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, "supervoxel_normals");

  // moved from here "getting supervoxel adjacency"

  //pcl::console::print_highlight ("JUST BEFORE THE ADJACENCY FOR LOOP\n");
  //To make a graph of the supervoxel adjacency, we need to iterate through the supervoxel adjacency multimap
  for (SupervoxelAdjacency::const_iterator label_itr = supervoxel_adjacency.cbegin ();
      label_itr != supervoxel_adjacency.cend (); )
  {
    //First get the label
    KeyT supervoxel_label = label_itr->first;
    //Now get the supervoxel corresponding to the label
    pcl::Supervoxel<PointT>::Ptr supervoxel = supervoxel_clusters.at (supervoxel_label);

    //Now we need to iterate through the adjacent supervoxels and make a point cloud of them
    PointCloudT adjacent_supervoxel_centers;
    for (auto adjacent_itr = supervoxel_adjacency.equal_range (supervoxel_label).first; adjacent_itr!=supervoxel_adjacency.equal_range (supervoxel_label).second; ++adjacent_itr)
    {
      pcl::Supervoxel<PointT>::Ptr neighbor_supervoxel = supervoxel_clusters.at (adjacent_itr->second);
      adjacent_supervoxel_centers.push_back (neighbor_supervoxel->centroid_);
    }
    //Now we make a name for this polygon
    std::stringstream ss;
    ss << params.filename << "_supervoxel_" << supervoxel_label;
    //This function is shown below, but is beyond the scope of this tutorial - basically it just generates a "star" polygon mesh from the points given
    addSupervoxelConnectionsToViewer (supervoxel->centroid_, adjacent_supervoxel_centers, ss.str (), viewer);
    //Move iterator forward to next label
    label_itr = supervoxel_adjacency.upper_bound (supervoxel_label);
  }


}


void
addSupervoxelConnectionsToViewer (PointT const &supervoxel_center,
                                  PointCloudT const &adjacent_supervoxel_centers,
                                  std::string supervoxel_name,
                                  pcl::visualization::PCLVisualizer::Ptr & viewer)
{
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
  vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();

  //Iterate through all adjacent points, and add a center point to adjacent point pair
  for (auto adjacent_itr = adjacent_supervoxel_centers.begin (); adjacent_itr != adjacent_supervoxel_centers.end (); ++adjacent_itr)
  {
    points->InsertNextPoint (supervoxel_center.data);
    points->InsertNextPoint (adjacent_itr->data);
  }
  // Create a polydata to store everything in
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
  // Add the points to the dataset
  polyData->SetPoints (points);
  polyLine->GetPointIds  ()->SetNumberOfIds(points->GetNumberOfPoints ());
  for(unsigned int i = 0; i < points->GetNumberOfPoints (); i++)
    polyLine->GetPointIds ()->SetId (i,i);
  cells->InsertNextCell (polyLine);
  // Add the lines to the dataset
  polyData->SetLines (cells);
  viewer->addModelFromPolyData (polyData,supervoxel_name);
}
