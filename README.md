## Point Cloud Registration
_Based on Huang et al, 2017 (1)_

 * Input: two point clouds
 * For each point cloud:
    * Supervoxel Clustering following Papon et al, 2013 (2)
    * For each cluster:
      * Compute the ESF descriptors of the cluster following Wohlkinger & Vincze, 2011 (3)
    * For each edge between adjacent clusters:
      * Compute angle_x, angle_y, angle_z, distance following Huang et al, 2017 (1)
 * Match the two resulting graphs, with vertices labeled by ESF and edges labeled by angles and distance
 * Apply RANSAC algorithm (4) to the point clouds
 * Apply ICP algorithm (5, 6) to the point clouds


### Dependencies

 * Point cloud library http://pointclouds.org/

### References

 (1) "A Systematic Approach for Cross-Source Point Cloud Registration by Preserving Macro and Micro Structures", Huang, Zhang, Fan, Wu, Yuan, IEEE Transactions on Image Processing, 2017

 (2) "Voxel Cloud Connectivity Segmentation - Supervoxels for Point Clouds", Papon, Abramov, Schoeler, Wörgötter, IEEE Conference on Computer Vision and Pattern Recognition, 2013

 (3) "Ensemble of Shape Functions for 3D Object Classification", Wohlkinger, Vincze, IEEE International Conference on Robotics and Biomimetics, 2011

 (4) "Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography.", Fischler, Bolles,  Communications of the ACM, 1981

 (5) "A Method for Registration of 3-D Shapes", Besl, McKay, IEEE Trans. Pattern Anal. Mach. Intell., 1992

 (6) "Object modeling by registration of multiple range images", Chen, Medioni, 1991
 
