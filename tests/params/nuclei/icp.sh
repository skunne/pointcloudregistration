mkdir -p icp/pointclouds

for clouddest in pointclouds/nuclei*.pcd
do
    ../../icpransac/icpransac_gettransform pointclouds/nuclei.pcd $clouddest icp/"$clouddest"_transform.csv
done

rm -f -- icp/distances.txt
for clouddest in pointclouds/nuclei*.pcd
do
    echo -n $clouddest '    ' >> icp/distances.txt
    ../../../scripts_python/distance_matched_pointclouds_pcd.py icp/"$clouddest"_transform.csv pointclouds/nuclei.pcd $clouddest >> icp/distances.txt
done
