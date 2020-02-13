## Testing RanSaC to remove "outlier" nodes after graph-matching
The purpose of this subfolder is to test `pcl::SampleConsensusModelRegistration` and `pcl::RandomSampleConsensus` from the PCL library.

Take two matched point clouds and use RanSaC (1) to remove points that are not well-matched. Assumes a rigid geometric transform between the two sets of nodes in 3d; ie, the transformation should be a rotation plus a translation.

### Install

Make sure PCL and Eigen are installed. Make sure the `INCLUDES` variable in `./Makefile` references the correct folders on your system. Then simply run `make`. This will create an executable named `a.out`.
~~~~
make
~~~~

### Usage

Display a hemisphere with lots of noise:
~~~~
./a.out -s
~~~~

Remove noise by using RanSaC to compare to a hemisphere without noise:
~~~~
./a.out -sf
~~~~

Display two point clouds from csv files:
~~~~
./a.out ../../maketests/matched_pointclouds/nuclei.pcd_src.csv ../../maketests/matched_pointclouds/nuclei.pcd_dst.csv
~~~~

Load two point clouds from csv files and remove outliers using RanSaC:
~~~~
./a.out -f ../../maketests/matched_pointclouds/nuclei.pcd_src.csv ../../maketests/matched_pointclouds/nuclei.pcd_dst.csv
~~~~


### References
(1) "Random sample consensus: a paradigm for model fitting with applications to image analysis and automated cartography.", Fischler, Bolles,  Communications of the ACM, 1981

(2) "PCL Tutorials: How to use Random Sample Consensus model" <http://pointclouds.org/documentation/tutorials/random_sample_consensus.php>
