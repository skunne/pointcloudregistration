for f in heart #nuclei valve
do
    for r in translated10 rotatedpi8 rotatedpi4
    do
        ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd ../params/"$f"/pointclouds/"$f"_"$r".pcd out.txt
        python3 test_gm_on_clusters.py src_out.txt dst_out.txt mat.txt transforms/"$r".csv Figure_"$f"_"$r".png
    done
done
