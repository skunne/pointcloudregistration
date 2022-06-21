Subfolders to test and validate the algorithm and various subparts of the algorithm.

### esf
Test Wohlkinger's ESF algorithm by comparing the ESF of various pointclouds.

### frankwolfe
Test graph-matching with Frank-Wolfe's algorithm for quadratic programming.

### ipopt
Test graph-matching with IpOpt library for nonlinear programming.

### qp
Test graph-matching with CGAL library for mixed-integer programming. It was concluded that CGAL was too slow for this step of the algorithm. Use Frank-Wolfe or IpOpt instead.

### icpransac
Run the ICP/RanSaC algorithm to register to point clouds, so that we can compare our algorithm's registrations with ICP/RanSaC's registrations.

### maketests
Run the algorithm to register 3 pointclouds with modified versions of themselves, with known modifications, including rotations, resampling and cropping. The computed transforms are compared with the known "groundtruth" transforms, and with transforms computed by ICP/RanSaC.

### params
Test the influence of parameters "seeds" and "voxels" of J. Papon's supervoxel clustering algorithm on the quality of the registration.

### ransac
Test the importance of the "remove outliers nodes" step of the algorithm after the graph-matching.
