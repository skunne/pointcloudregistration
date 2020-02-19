## Point Cloud Registration
Test graph matching using the Nonlinear Programming solver from library IpOpt.

### Dependencies

 * Eigen <http://eigen.tuxfamily.org/>
 * Ipopt <https://coin-or.github.io/Ipopt/>

### Install

Edit the lines `INCLUDES=...` and `LIBPATHS=...` in `./Makefile` to reflect the location at which you installed the library Ipopt. Then compile by simply typing `make` in the console.
~~~~
make
~~~~

### Usage

~~~~
./test_ipopt
~~~~

### References

The example at <https://coin-or.github.io/Ipopt/INTERFACES.html> was a great help.
