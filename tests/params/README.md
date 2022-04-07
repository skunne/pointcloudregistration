Testing params "voxels" and "seeds" of the algorithm.

Compile :

    cmake .
    make

Run all computations :

    ./computeallmatchings.sh
    ./computealltransforms.sh
    ./computealldistances.sh

Commands in those three .sh files have been generated using python script make_cmd.py

To get the 3d plot of distance with respect to voxels,seed:

    python plot_distances.py
