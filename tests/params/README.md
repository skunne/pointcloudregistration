Testing params "voxels" and "seeds" of the algorithm.

Compile :

    cmake .
    make

Run all computations :

    cd heart/
    ./computeallmatchings.sh
    ./computealltransforms.sh
    ./computealldistances.sh

Commands in those three .sh files have been generated using python script make_cmd.py

Or smaller test run:
    ./test_params heart/pointclouds/heart.meta heart/pointclouds/heart.pcd 30 150 heart/pointclouds/heart.pcd 30 150

    java -jar ../maketests/cli_tools/compute_transformation/target/compute_transformation-0.1.0-SNAPSHOT.jar --source-dataset /SCRATCH-BIRD/users/skunne/matched_heart/pointclouds/heart.pcd_src_v30s150v30s150.csv --target-dataset /SCRATCH-BIRD/users/skunne/matched_heart/pointclouds/heart.pcd_dst_v30s150v30s150.csv --transformation-model RIGID >> /SCRATCH-BIRD/users/skunne/matched_heart/transforms/v30s150v30s150.txt

    ../../scripts_python/distance_matched_pointclouds_csv.py /SCRATCH-BIRD/users/skunne/matched_heart/transforms/v30s150v30s150.txt /SCRATCH-BIRD/users/skunne/matched_heart/pointclouds/heart.pcd_src_v30s150v30s150.csv /SCRATCH-BIRD/users/skunne/matched_heart/pointclouds/heart.pcd_dst_v30s150v30s150.csv

* First command `./computeallmatchings.sh` or `./test_params ...`: compute point-to-point matching between two pointclouds, and output two files with the points listed in matching order.
* Second command `./computealltransforms.sh` or `java -jar .../compute_transformation-0.1.0-SNAPSHOT.jar ...`: compute geometric transformation between the two pointclouds and output one file with the 4x4 matrix representing the affine transformation in homogenous coordinates.
* Third command `./computealldistances.sh` or `../scripts_python/distance_matched_pointclouds_csv.py ...`: compute average distance between the two registered pointclouds.

Output files for the first two steps are in folders `/SCRATCH-BIRD/users/skunne/matched_heart/pointclouds` and `/SCRATCH-BIRD/users/skunne/matched_heart/transforms/`. Output file for the third step is `distances.txt`.



To get the 3d plot of distance with respect to voxels,seed:

    python plot_distances.py
