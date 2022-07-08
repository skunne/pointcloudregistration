mkdir -p icp/pointclouds

for clouddest in pointclouds/valve*.pcd
do
    ../../icpransac/icpransac_gettransform pointclouds/valve.pcd $clouddest icp/"$clouddest"_transform.csv
done

rm -f -- icp/distances.txt
for clouddest in pointclouds/valve*.pcd
do
    echo -n $clouddest '    ' >> icp/distances.txt
    ../../../scripts_python/distance_matched_pointclouds_pcd.py icp/"$clouddest"_transform.csv pointclouds/valve.pcd $clouddest >> icp/distances.txt
done
