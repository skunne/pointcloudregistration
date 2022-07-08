mkdir -p icp/pointclouds

for clouddest in pointclouds/heart*.pcd
do
    ../../icpransac/icpransac_gettransform pointclouds/heart.pcd $clouddest icp/"$clouddest"_transform.csv
done

rm -f -- icp/distances.txt
for clouddest in pointclouds/heart*.pcd
do
    echo -n $clouddest '    ' >> icp/distances.txt
    ../../../scripts_python/distance_matched_pointclouds_pcd.py icp/"$clouddest"_transform.csv pointclouds/heart.pcd $clouddest >> icp/distances.txt
done
