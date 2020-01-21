## Testing ESF descriptors
Compare two or more point clouds using ESF descriptors.

### Install

~~~~
cmake .
make
~~~~

### Usage
Give metadata files as commandline arguments.

If you use the metadata files from `../../metadata`, then it is important to run binary from `../../`

The binary will output the matrix of distances between the ESF descriptors of every point cloud.

~~~~
cd ../..
./tests/esf/test_esf metadata/big1.meta metadata/rot.meta
./tests/esf/test_esf metadata/*
~~~~
