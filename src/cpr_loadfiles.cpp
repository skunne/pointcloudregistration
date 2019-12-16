
#include <pcl/io/vtk_lib_io.h> //VTK include needed to input vtk cloud file
#include <pcl/io/pcd_io.h>     // pcl::io::loadPCDFile
#include <pcl/io/ply_io.h>     // pcl::PLYReader
#include "cpr_main.h"
#include "cpr_processedpointcloud.h"
#include "cpr_loadfiles.h"

int ProcessedPointCloud::loadFile(void) //char const *filename, bool is_pc)//, PointCloudT::Ptr cloud)
{
  if (params.is_pcd)
    return loadPCDFile(params.filename.c_str(), cloud);
  else if (params.is_vtk)
    return loadVTKFile(params.filename.c_str(), cloud);
  else if (params.is_ply)
    return loadPLYFile(params.filename.c_str(), cloud);
  else
  {
    pcl::console::print_error("Unknown file format for pointcloud file %s\n", params.filename.c_str());
    return 7;
  }
}

int cpr_loadFile(char const *filename, bool is_pcd, PointCloudT::Ptr cloud)
{
  if (is_pcd)
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
  pcl::console::print_highlight ("Loading point cloud from vtk file %s\n", filename);
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
    return errorLoadingFile(".vtk cloud", filename);
  pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);

  return (0);
}

int loadPCDFile(char const *filename, PointCloudT::Ptr cloud)
{
  pcl::console::print_highlight ("Loading point cloud from .pcd file %s\n", filename);
  if (pcl::io::loadPCDFile<PointT> (filename, *cloud))
    return errorLoadingFile(".pcd cloud", filename);

  return (0);
}

int loadPLYFile(char const *filename, PointCloudT::Ptr cloud)
{
  pcl::PLYReader reader;
  pcl::console::print_highlight ("Loading point cloud from .ply file %s\n", filename);
  if (reader.read(filename,*cloud))
    return errorLoadingFile(".ply cloud", filename);

  return (0);
}
