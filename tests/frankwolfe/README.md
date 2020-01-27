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

If `test_with_pointclouds(argc, argv)` is uncommented in test_main.cpp, then the binary must be run with two metadata files as argument. Please note that when using the metadata files from the `metadata` folder, the binary should be run from the root folder of the project, because the filepaths written in those files are all relative to the root folder.

~~~~
./test_frankwolfe

cd ../..
./tests/frankwolfe/test_frankwolfe metadata/big1.meta metadata/rot.meta
~~~~
