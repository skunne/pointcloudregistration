### Metadata files

The files in this subfolder are configuration files to be used by the program.

Each metadata file must consist of 7 lines :

* the path to the file describing a point cloud
* five lines with parameters for the supervoxel clustering algorithm
* the path to the output file where the adjacency matrix of the resulting graph will be written

In all metadata files in this folder, the file paths are given relative to the root of the project. Hence, when running a binary from the project with one of these metadata files as argument, the binary must be run from the root of the project.
