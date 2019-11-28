
#include "main.h"

int load_file(char const *filename, bool is_pcd, PointCloudT::Ptr cloud)
{
  if (is_pcd)
    return load_pcdfile(filename, cloud);
  else
    return load_vtkfile(filename, cloud);
}

int load_vtkfile(char const *filename, PointCloudT::Ptr cloud)
{
  //PointCloudT cloud (new PointCloudT);
  vtkPolyData *polydata;
  pcl::console::print_highlight ("Loading point cloud from .vtk file...\n");
  // load vtk file
  vtkSmartPointer<vtkGenericDataObjectReader> vtkReader =
      vtkSmartPointer<vtkGenericDataObjectReader>::New();
    vtkReader->SetFileName(filename);
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

  return (0);
}

int load_pcdfile(char const *filename, PointCloudT::Ptr cloud)
{
  pcl::console::print_highlight ("Loading point cloud from .pcd file...\n");
  if (pcl::io::loadPCDFile<PointT> (filename, *cloud))
  {
    pcl::console::print_error ("Error loading cloud file!\n");
    return (1);
  }

  return (0);
}
