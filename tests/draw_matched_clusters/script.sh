for f in heart nuclei valve
do
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd ../params/"$f"/pointclouds/"$f".pcd out.txt
    python3 test_gm_on_clusters.py src_out.txt dst_out.txt Figure_"$f".png
done
