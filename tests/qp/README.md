## Testing graph-matching with the CGAL mixed-integer quadratic solver
Test for the class GraphMatchingCgal defined in ../../src/cpr_graphmatching_cgal.cpp
Using this class, the graph-matching step from Huang's algorithm is replaced with the mixed-integer quadratic solver from CGAL.

Conclusion from this test: the mixed-integer solver is way too slow for this task.

### Install

~~~~
cmake .
make
~~~~

### Usage
Give metadata files as commandline arguments.

If you use the metadata files from `../../metadata`, then it is important to run binary from `../../`

~~~~
cd ../..
./tests/qp/test_qp metadata/big1.meta metadata/rot.meta
~~~~
