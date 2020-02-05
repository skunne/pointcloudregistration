## Testing the registration algorithm
Test the algorithm by registering a given point cloud with transformed copies of itself.

### Outline
 * Input a point cloud from the file given on line `NAME=...` in `./Makefile`.
 * Generate 8 new point clouds by shuffling, cropping, thinning and rotating the original point cloud.
 * Run the algorithm to register the original point cloud against each of the 9 point clouds.
 * For each of the 9 point clouds, output two csv files in the subfolder `registered_pointclouds`. Each csv file represents a point cloud; the points are sorted to reflect the matching.
 * TODO: Find the geometric transform resulting from the matching
 * TODO: Compare the found geometric transform with the known geometric transform

### Usage
Optionally change the name of the point cloud file on line `NAME=...` in `./Makefile`.

Then run everything with the simple command:
~~~~
make
~~~~
