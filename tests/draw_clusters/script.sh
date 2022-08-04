for f in heart nuclei valve
do
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd out.txt
    python3 draw_clusters.py out.txt Figure_"$f".png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_translated10.pcd out.txt
    python3 draw_clusters.py out.txt Figure_"$f"_translated10.png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_rotatedpi8.pcd out.txt
    python3 draw_clusters.py out.txt Figure_"$f"_rotatedpi8.png
done
