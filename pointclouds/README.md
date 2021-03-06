## A few point cloud files
### .vtk files
 * point clouds in .vtk files generated using Icy plugin BinaryToPointROI:
   * http://icy.bioimageanalysis.org/plugin/ec-clem-autofinder/

### milk_cartoon_all_small_clorox.pcd
 * point cloud from the pcl library git repository:
   * https://github.com/PointCloudLibrary/data/blob/master/tutorials/correspondence_grouping/milk_cartoon_all_small_clorox.pcd?raw=true

### snowman.pcd, nuclei.pcd, beads.pcd
 * artificial point clouds generated with the scripts makeSnowman.py, makeNuclei.py and makeBeads.py in subfolder ../scripts_python/makepointcloud/

### nuclei_hole.pcd, beads_minus000333.pcd, beads_minus800X5X.pcd
 * artificial point clouds generated by removing points from nuclei.pcd and beads.pcd with the script dig_hole.py in ../scripts_python.makepointcloud/
   * nuclei_hole.pcd: removed all points in the cube [0,3]\*[0,3]\*[0,3]
   * beads_minus000555.pcd: removed all points in the cube [0,5]\*[0,5]\*[0,5]
   * beads_minus800X5X.pcd: removed all points in the cuboid [8,10]\*[0,5]\*[0,10]

### .ply files
 * the files Armadillo.ply and dragon_vrip_res2.ply referenced by ../metadata/dragon.meta and ../metadata/armadillo.meta can be downloaded from the Standford 3D Scanning Repository:
   * http://graphics.stanford.edu/data/3Dscanrep/
