

#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_visualisation.h"

void make_colour(int n, double &r, double &g, double &b)
{
  if (n == 0)
  {
    r = g = b = 80.0;    // replace black with grey for visibility
  }
  else
  {
    r = (n >> 2) * 255.0;         // 4 bit
    g = ((n >> 1) & 1) * 255.0;   // 2 bit
    b = (n & 1) * 255.0;          // unit bit
  }
}

void ProcessedPointCloud::addSomeColours(pcl::visualization::PCLVisualizer::Ptr viewer, std::vector<int> nodes)
{
  int i = 0;
  for (std::vector<int>::const_iterator nodes_itr = nodes.cbegin(); nodes_itr != nodes.cend(); ++nodes_itr)
  {
    std::stringstream ss;
    ss << params.filename << i;
    double r,g,b;
    make_colour(i, r, g, b);
    pcl::console::print_error("%d -> %d, %f, %f, %f.\n", *nodes_itr, i, r, g, b);
    viewer->addSphere(supervoxel_clusters[*nodes_itr]->centroid_, 3.0, r, g, b, ss.str());
    ++i;
  }
  /*if (params.filename == "pointclouds/biggerpointcloud.vtk")
  {
    viewer->addSphere(supervoxel_clusters[0]->centroid_, 3.0, 255.0, 0.0, 0.0, "big1_sphere0");
    viewer->addSphere(supervoxel_clusters[9]->centroid_, 3.0, 0.0, 255.0, 0.0, "big1_sphere9");
    viewer->addSphere(supervoxel_clusters[19]->centroid_, 3.0, 0.0, 0.0, 255.0, "big1_sphere19");
    viewer->addSphere(supervoxel_clusters[49]->centroid_, 3.0, 255.0, 255.0, 0.0, "big1_sphere49");
    viewer->addSphere(supervoxel_clusters[99]->centroid_, 3.0, 255.0, 0.0, 255.0, "big1_sphere99");
  }
  else if (params.filename == "pointclouds/rotated30degrees_1um.vtk")
  {
    viewer->addSphere(supervoxel_clusters[20]->centroid_, 3.0, 255.0, 0.0, 0.0, "rot_sphere20");
    viewer->addSphere(supervoxel_clusters[10]->centroid_, 3.0, 0.0, 255.0, 0.0, "rot_sphere10");
    viewer->addSphere(supervoxel_clusters[13]->centroid_, 3.0, 0.0, 0.0, 255.0, "rot_sphere13");
    viewer->addSphere(supervoxel_clusters[128]->centroid_, 3.0, 255.0, 255.0, 0.0, "rot_sphere128");
    viewer->addSphere(supervoxel_clusters[78]->centroid_, 3.0, 255.0, 0.0, 255.0, "rot_sphere78");
    //viewer->addSphere(supervoxel_clusters[20]->centroid_, 3.0, 255.0, 0.0, 0.0, "rot_sphere20");
  }*/
}

//pcl::visualization::PCLVisualizer::Ptr
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

  //source.addSomeColours(viewer);
  //dest.addSomeColours(viewer);

  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
  }

  //return viewer;
}

void ProcessedPointCloud::addToViewer(pcl::visualization::PCLVisualizer::Ptr viewer)
{
  PointCloudT::Ptr voxel_centroid_cloud = super.getVoxelCentroidCloud ();
  {  // scope for stringstream ss
    std::stringstream ss;
    ss << params.filename << "_voxelcentroids";
    viewer->addPointCloud (voxel_centroid_cloud, ss.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2.0, ss.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.95, ss.str());
  }  // end scope for stringstream ss

  PointLCloudT::Ptr labeled_voxel_cloud = super.getLabeledVoxelCloud ();
  {
    std::stringstream ss;
    ss << params.filename << "_labeledvoxels";
    viewer->addPointCloud (labeled_voxel_cloud, ss.str());
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_OPACITY,0.8, ss.str());
  }

  //We have this disabled so graph is easy to see, uncomment to see supervoxel normals
  //PointNCloudT::Ptr sv_normal_cloud = super.makeSupervoxelNormalCloud (supervoxel_clusters);
  //{
  //  std::stringstream ss;
  //  ss << params.filename << "_supervoxel_normals";
  //  viewer->addPointCloudNormals<PointNormal> (sv_normal_cloud,1,0.05f, ss.str().c_str());
  //}

  // moved from here "getting supervoxel adjacency"

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

    //ss << "_sphere";
    //viewer->addSphere(supervoxel->centroid_, 3.0, 255.0, 0.0, 0.0, ss.str());

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
