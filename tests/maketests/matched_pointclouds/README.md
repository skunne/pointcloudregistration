## Registered sets of nodes

### File format
Each csv file has one field `id` and three fields `dimension_1`, `dimension_2`, `dimension_3`.

The lists of points in one file correspond to the nodes of the graph after application of the algorithm to the corresponding point cloud.

All the files ending in `_src.csv` correspond to the same point cloud `nuclei.pcd`. The files ending in `_dst.csv` correspond to a variation of that point cloud (by cropping, rotating, etc).

The two files `xxx_src.csv` and `xxx_dst.csv` together form the result of the graph matching: every node with a given `id` in `xxx_src.csv` is matched to the node with same `id` in `xxx_dst.csv`.

### Visualisation
The script `visualise.sh` is there to visualise the point clouds. Calling `./visualise.sh filename.csv` will process the csv file `filename.csv`, generate a temporary file in a format closer to the pcd format, and run `../../../scripts_python/other/show_pcd_file.py` on the temporary file to display the point cloud. The resulting figure is saved in the file `figure.png`.
