
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_loadfiles.h"

int ProcessedPointCloud::loadFile(void) //char const *filename, bool is_pc)//, PointCloudT::Ptr cloud)
{
  if (params.is_pcd)
    return loadPCDFile(filename, cloud);
  else
    return loadVTKFile(filename, cloud);
}

int errorLoadingFile(char const *type, char const *name)
{
  pcl::console::print_error ("Error loading ");
  pcl::console::print_error (type);
  pcl::console::print_error (" file: ");
  pcl::console::print_error(name);
  pcl::console::print_error("\n");
  return (1);
}

int loadVTKFile(char const *filename, PointCloudT::Ptr cloud)
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
    return errorLoadingFile("vtk cloud", filename);
  pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

  return (0);
}

int loadPCDFile(char const *filename, PointCloudT::Ptr cloud)
{
  pcl::console::print_highlight ("Loading point cloud from .pcd file...\n");
  if (pcl::io::loadPCDFile<PointT> (filename, *cloud))
    return errorLoadingFile("pcd cloud", filename);

  return (0);
}
