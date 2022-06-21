### Usage:

    /icpransac_gettransform <cloudsource.pcd> <clouddest.pcd> <transform.csv>

Apply ICP/RanSaC to two point clouds to find the rigid transform from source to dest.

The resulting 4x4 matrix is written to `transform.csv` and describes the transformation in homogeneous coordinates.
