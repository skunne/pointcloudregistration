### Transforms
4x4 matrices representing the transformation between nuclei.pcd and nuclei_rotated.pcd in homogenous coordinates.
 * true.csv True transform used to build nuclei_rotated.pcd from nuclei.pcd
 * graphmatching_without_ransac.csv Transform calculated with Huang's algorithm, using Ipopt solver, but without the ransac "remove outliers" step.
 * graphmatching_with_ransac.csv Transform calculated with Huang's algorithm, using Ipopt solver, after the ransac "remove outliers" step
 * icpransac.csv Transform calculated using ICP/Ransac.

