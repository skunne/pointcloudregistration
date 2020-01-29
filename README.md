## Point Cloud Registration
Based on _Huang et al, 2017_ (1). Using the Point Cloud Library, and in particular code from J. Papon and W. Wohlkinger.

 * Input: two point clouds
 * For each point cloud:
    * Supervoxel Clustering following _Papon et al, 2013_ (2)
    * For each cluster:
      * Compute the 640 ESF descriptors of the cluster following _Wohlkinger & Vincze, 2011_ (3)
    * For each edge between adjacent clusters:
      * Compute the 4 descriptors `(angle_x, angle_y, angle_z, distance)` following _Huang et al, 2017_ (1)
 * Build vertex similarity matrix using distance between ESF descriptors
 * Build edge similarity matrix using distance between 4-descriptors
 * Match the two resulting graphs using Frank-Wolfe algorithm for quadratic programming (5)
 * TODO Apply RANSAC algorithm (6) to the graph vertices to remove outliers
 * TODO Find a rigid geometric transform to map the first set of graph vertices to the second set
 * TODO Apply ICP algorithm (7, 8) to the graph vertices as a final refinement


### Dependencies

 * Point Cloud Library <http://pointclouds.org/>
 * Eigen <http://eigen.tuxfamily.org/>
 * GLPK <https://en.wikibooks.org/wiki/GLPK/>

### Install

~~~~
git clone https://github.com/skunne/pointcloudregistration.git
cd pointcloudregistration
cmake .
make
~~~~

### Usage

The executable expects two arguments. Each argument is the name of a metadata file. A metadata file contains the name of a pointcloud file in `.vtk`, `.pcd` or `.ply` format, along with parameters for the supervoxel clustering algorithm. Examples of metadata files can be found in the `metadata/` subfolder, and the corresponding point cloud files in the `pointclouds/` subfolder. For an overview of the parameters, please refer to http://pointclouds.org/documentation/tutorials/supervoxel_clustering.php and to the corresponding paper (2).

Example use:
~~~~
./register_pointclouds metadata/big1.meta metadata/rot.meta
~~~~

### References

 (1) "A Systematic Approach for Cross-Source Point Cloud Registration by Preserving Macro and Micro Structures", Huang, Zhang, Fan, Wu, Yuan, IEEE Transactions on Image Processing, 2017

 (2) "Voxel Cloud Connectivity Segmentation - Supervoxels for Point Clouds", Papon, Abramov, Schoeler, Wörgötter, IEEE Conference on Computer Vision and Pattern Recognition, 2013

 (3) "Ensemble of Shape Functions for 3D Object Classification", Wohlkinger, Vincze, IEEE International Conference on Robotics and Biomimetics, 2011

 (4) "A Path Following Algorithm for the Graph Matching Problem", Zaslavskiy, Bach, Vert, IEEE Transactions on Pattern Analysis and Machine Intelligence, 2009

 (5) "An Algorithm for Quadratic Programming", Frank, Wolfe, Naval research logistics quarterly, 1956

 (6) "Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography.", Fischler, Bolles,  Communications of the ACM, 1981

 (7) "A Method for Registration of 3-D Shapes", Besl, McKay, IEEE Trans. Pattern Anal. Mach. Intell., 1992

 (8) "Object modeling by registration of multiple range images", Chen, Medioni, 1991
