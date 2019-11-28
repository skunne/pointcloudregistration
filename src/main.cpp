
#include "main.h"
#include "params.h"


int
main (int argc, char ** argv)
{
  struct Params params;
  set_params(argc, argv, &params);

  PointCloudT::Ptr cloud (new PointCloudT);

  if (pcl::console::find_switch (argc, argv, "--pcd"))
  {
    pcl::console::print_highlight ("Loading point cloud from .pcd file...\n");
    if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud))
    {
      pcl::console::print_error ("Error loading cloud file!\n");
      return (1);
    }
  }
  else
  {
    //PointCloudT cloud (new PointCloudT);
    vtkPolyData *polydata;
    pcl::console::print_highlight ("Loading point cloud from .vtk file...\n");
    // load vtk file
    vtkSmartPointer<vtkGenericDataObjectReader> vtkReader =
        vtkSmartPointer<vtkGenericDataObjectReader>::New();
      vtkReader->SetFileName(argv[1]);
    vtkReader->Update();
    if(vtkReader->IsFilePolyData())
    {
      pcl::console::print_highlight ("Point cloud loaded.\n");
      polydata = vtkReader->GetPolyDataOutput();
      //std::cout << "Point cloud has " << polydata->GetNumberOfPoints() << " points." << std::endl;
      pcl::console::print_info ("Point cloud has %d points.\n", polydata->GetNumberOfPoints());
    }
    else
    {
      pcl::console::print_error ("Error loading vtk cloud file!\n");
      return (1);
    }
    pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
  }





  //////////////////////////////  //////////////////////////////
  ////// This is how to use supervoxels
  //////////////////////////////  //////////////////////////////

  SupervoxelClusters supervoxel_clusters;
  SupervoxelAdjacency supervoxel_adjacency;
  pcl::SupervoxelClustering<PointT> super (params.voxel_resolution, params.seed_resolution);
  perform_clustering(cloud, super, &params, supervoxel_clusters, supervoxel_adjacency);

  //////////////////////////////  //////////////////////////////
  ////// ESF descriptors calculation
  //////////////////////////////  //////////////////////////////

  ESFDescriptors esf_descriptors;
  EdgeDescriptors edge_descriptors;
  calculate_descriptors(supervoxel_clusters, supervoxel_adjacency, esf_descriptors, edge_descriptors);

  //////////////////////////////  //////////////////////////////
  ////// This is the visualisation part
  //////////////////////////////  //////////////////////////////

  visualisation(super, supervoxel_clusters, supervoxel_adjacency);
  return (0);
}
