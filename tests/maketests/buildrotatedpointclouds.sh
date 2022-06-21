set -v

echo "1,0,0,0
0,1,0,0
0,0,1,0
0,0,0,1" > 'transforms/valve.pcd_true.csv'

list_of_angles=(0.0 0.3141592653589793  0.6283185307179586  0.9424777960769379  1.2566370614359172  1.5707963267948966)

for angle in ${list_of_angles[*]}; do
  /usr/bin/python3 ../../scripts_python/apply_transform.py pointclouds/valve.pcd pointclouds/valve_r"$angle".pcd $angle transforms/valve_r"$angle".pcd_true.csv
  /usr/bin/python3 ../../scripts_python/pcd_to_csv.py pointclouds/valve_r"$angle".pcd pointclouds/valve_r"$angle".csv
done
