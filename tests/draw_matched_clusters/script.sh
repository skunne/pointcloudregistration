set -x

for f in heart nuclei valve
do
    for r in id translated10 rotatedpi8 rotatedpi4
    do
        ./print_voxels ../params/"$f"/pointclouds/"$f".meta ../params/"$f"/pointclouds/"$f".pcd ../params/"$f"/pointclouds/"$f"_"$r".pcd out.txt
        mv esf_similarity_matrix esf_similarity_matrix_"$f"_"$r"
        python3 test_gm_on_clusters.py src_out.txt dst_out.txt mat.txt transforms/"$r".csv Figure_"$f"_"$r".png
    done
done

for f in armadillo
do
    # ./print_voxels ../../metadata/armadillo.meta ../../pointclouds/armadillo.pcd ../../pointclouds/armadillo.pcd out.txt
    # mv esf_similarity_matrix esf_similarity_matrix_armadillo_id
    # python3 test_gm_on_clusters.py src_out.txt dst_out.txt mat.txt transforms/id.csv Figure_armadillo_id.png
    for r in translated10 rotatedpi8 resampled_uniform resampled_random
    do
        ./print_voxels ../../metadata/"$f".meta ../../pointclouds/"$f".pcd ../../pointclouds/"$f"_"$r".pcd out.txt
        mv esf_similarity_matrix esf_similarity_matrix_"$f"_"$r"
        python3 test_gm_on_clusters.py src_out.txt dst_out.txt mat.txt transforms/"$r".csv Figure_"$f"_"$r".png
    done
done
