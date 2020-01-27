## Testing graph-matching on very small graphs
Match two small graphs using the Frank-Wolfe algorithm for quadratic programming.

### Install

 * Optionally comment out or uncomment tests in main_test.cpp
 * Then compile with:
 
~~~~
cmake .
make
~~~~

### Usage
For most of the tests, the binary can be run simply.

If `test_with_pointclouds(argc, argv)` is uncommented in test_main.cpp, then the binary must be run with two metadata files as argument. In that case, move to `../../` and call the binary with files from the `metadata` folder.

~~~~
./test_frankwolfe

cd ../..
./tests/frankwolfe/test_frankwolfe metadata/big1.meta metadata/rot.meta
~~~~
