for f in heart nuclei valve
do
    print_voxels ../params/$f/pointclouds/$f.meta ../params/$f/pointclouds/$f.pcd out_$f.txt
done

for f in heart nuclei valve
do
    python3 draw_clusters.py out_$f.txt Figure_$f.png
done
