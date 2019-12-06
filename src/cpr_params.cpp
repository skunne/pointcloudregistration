
#include <algorithm>
#include <fstream>
#include <sstream>
#include <string>
#include "cpr_main.h"
#include "cpr_loadfiles.h"    //errorLoadingFile()
#include "cpr_params.h"

Params::Params(char const *metadata_filename) : error(0)
{
  std::ifstream meta(metadata_filename);

  if (!meta)
  {
    error = errorLoadingFile("metadata", metadata_filename);
    return ;
  }

  std::string wrong_line;
  std::string line;
  std::getline(meta, line);
  std::istringstream iss(line);
  std::string field;
  std::string pointcloud_filename;
  if (!(iss >> field >> pointcloud_filename) || field != "filename")
  {
    error = 1;
    wrong_line = line;
  }
  else
  {
    filename = pointcloud_filename.c_str();

    is_vtk = endsWith(filename, ".vtk");
    is_pcd = !is_vtk && endsWith(filename, ".pcd");

    while (std::getline(meta, line))
    {
      std::istringstream iss(line);
      float value;
      if (!(iss >> field >> value)) { break; } // error
      if (field == "voxel_resolution" || field == "v")
        voxel_resolution = value;
      else if (field == "seed_resolution" || field == "s")
        seed_resolution = value;
      else if (field == "color_importance" || field == "c")
        color_importance = value;
      else if (field == "spatial_importance" || field == "z")
        spatial_importance = value;
      else if (field == "normal_importance" || field == "n")
        normal_importance = value;
      else
      {
          error = 1;
          wrong_line = line;
      }
    }
  }
  if (error)
  {
    pcl::console::print_error("Error reading line from file ");
    pcl::console::print_error(metadata_filename);
    pcl::console::print_error("\n    ");
    pcl::console::print_error(line.c_str());
  }
  //return getParams(argc, argv, &params);
}

bool endsWith(const std::string &a, const std::string &b)
{
    if (b.size() > a.size()) return false;
    return std::equal(a.begin() + a.size() - b.size(), a.end(), b.begin());
}

int printUsage(char const *command)
{
  pcl::console::print_error("Syntax is: %s metadatafile1 metadatafile2\n", command);
  return 1;
}

int getParams(int argc, char const *const *argv, struct Params *params)
{
  if (argc < 2)
  {
    pcl::console::print_error ("Syntax is: %s <vtk-file> \n"
                                //" --nt Disables the single camera transform \n"
                                "  -v <voxel resolution>\n  -s <seed resolution>\n"
                                "  -c <color weight>\n  -z <spatial weight>\n"
                                "  -n <normal_weight>\n"
                                "Or: %s <pcd-file> --pcd [--nt] [...]\n", argv[0], argv[0]);
    return (1);
  }

  params->is_pcd = pcl::console::find_switch (argc, argv, "--pcd");

  //params->disable_transform = pcl::console::find_switch (argc, argv, "--nt");

  params->voxel_resolution = (params->is_pcd ? 0.008f : 10.0f);
  params->voxel_res_specified = pcl::console::find_switch (argc, argv, "-v");
  if (params->voxel_res_specified)
    pcl::console::parse (argc, argv, "-v", params->voxel_resolution);

  params->seed_resolution = (params->is_pcd ? 0.1f : 15.0f);
  params->seed_res_specified = pcl::console::find_switch (argc, argv, "-s");
  if (params->seed_res_specified)
    pcl::console::parse (argc, argv, "-s", params->seed_resolution);

  params->color_importance = 0.2f;
  if (pcl::console::find_switch (argc, argv, "-c"))
    pcl::console::parse (argc, argv, "-c", params->color_importance);

  params->spatial_importance = 0.4f;
  if (pcl::console::find_switch (argc, argv, "-z"))
    pcl::console::parse (argc, argv, "-z", params->spatial_importance);

  params->normal_importance = 1.0f;
  if (pcl::console::find_switch (argc, argv, "-n"))
    pcl::console::parse (argc, argv, "-n", params->normal_importance);

  return (0);
}
