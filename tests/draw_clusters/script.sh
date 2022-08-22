for f in heart nuclei valve
do
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd out_"$f"_id.txt
    python3 draw_clusters.py out_"$f"_id.txt Figure_"$f".png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_translated10.pcd out_"$f"_translated10.txt
    python3 draw_clusters.py out_"$f"_translated10.txt Figure_"$f"_translated10.png
    ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f"_rotatedpi8.pcd out_"$f"_rotatedpi8.txt
    python3 draw_clusters.py out_"$f"_rotatedpi8.txt Figure_"$f"_rotatedpi8.png
done
