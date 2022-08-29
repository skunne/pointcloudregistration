set -x
for f in heart nuclei valve
do
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd out_"$f"_id.txt
    python3 draw_clusters.py out_"$f"_id.txt Figure_"$f".png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_translated10.pcd out_"$f"_translated10.txt
    python3 draw_clusters.py out_"$f"_translated10.txt Figure_"$f"_translated10.png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_rotatedpi8.pcd out_"$f"_rotatedpi8.txt
    python3 draw_clusters.py out_"$f"_rotatedpi8.txt Figure_"$f"_rotatedpi8.png
done

for f in armadillo
do
    ./print_voxels ../../metadata/armadillo.meta ../../pointclouds/armadillo.pcd out_armadillo_id.txt
    python3 draw_clusters.py out_armadillo_id.txt Figure_armadillo_id.png
    for r in translated10 rotatedpi8 resampled_uniform resampled_random
    do
        ./print_voxels ../../metadata/armadillo.meta ../../pointclouds/"$f"_"$r".pcd out_"$f"_"$r".txt
        python3 draw_clusters.py out_"$f"_"$r".txt Figure_"$f"_"$r".png
    done
done
