import sys    # argv, exit
from itertools import product

def print_usage_and_exit(argv):
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} [heart | nuclei | valve]'.format(sys.argv[0]))
    print('    Test different values for the "seed" and "voxel" parameters')
    print()
    sys.exit()

def get_args(argv):
    if len(argv) != 2 or argv[1] in {'-h', '--help'}:
        print_usage_and_exit(argv)
    else:
        return argv[1]

computetransform_dir='../maketests/cli_tools/compute_transformation/'
computetransform_jar = computetransform_dir + 'target/compute_transformation-0.1.0-SNAPSHOT.jar'

def computetransform_cmd(src, dst):
    return ' '.join([
        'java -jar',
        computetransform_jar,
        '--source-dataset',
        src,
        '--target-dataset',
        dst,
        '--transformation-model RIGID'
    ])

def main(argv):
    voxels, seeds = range(20,41,5), range(90,300,20)
    name = get_args(argv)
    with open('computeallmatchings.sh', 'w') as outf_matchings:
        print(
            '../test_params pointclouds/{}.meta ' + ' '.join('pointclouds/{}.pcd {} {} pointclouds/{}_rotatedpi4.pcd {} {}'.format(name, name, v,s, name, v,s) for v,s in product(voxels, seeds)),
            file=outf_matchings
        )

# './test_params pointclouds/heart.meta pointclouds/heart.pcd 20 90 pointclouds/heart_rotatedpi4.pcd 20 90 pointclouds/heart.pcd 20 110 pointclouds/heart_rotatedpi4.pcd 20 110 pointclouds/heart.pcd 20 130 pointclouds/heart_rotatedpi4.pcd 20 130 pointclouds/heart.pcd 20 150 pointclouds/heart_rotatedpi4.pcd 20 150 pointclouds/heart.pcd 20 170 pointclouds/heart_rotatedpi4.pcd 20 170 pointclouds/heart.pcd 20 190 pointclouds/heart_rotatedpi4.pcd 20 190 pointclouds/heart.pcd 20 210 pointclouds/heart_rotatedpi4.pcd 20 210 pointclouds/heart.pcd 20 230 pointclouds/heart_rotatedpi4.pcd 20 230 pointclouds/heart.pcd 20 250 pointclouds/heart_rotatedpi4.pcd 20 250 pointclouds/heart.pcd 20 270 pointclouds/heart_rotatedpi4.pcd 20 270 pointclouds/heart.pcd 20 290 pointclouds/heart_rotatedpi4.pcd 20 290 pointclouds/heart.pcd 25 90 pointclouds/heart_rotatedpi4.pcd 25 90 pointclouds/heart.pcd 25 110 pointclouds/heart_rotatedpi4.pcd 25 110 pointclouds/heart.pcd 25 130 pointclouds/heart_rotatedpi4.pcd 25 130 pointclouds/heart.pcd 25 150 pointclouds/heart_rotatedpi4.pcd 25 150 pointclouds/heart.pcd 25 170 pointclouds/heart_rotatedpi4.pcd 25 170 pointclouds/heart.pcd 25 190 pointclouds/heart_rotatedpi4.pcd 25 190 pointclouds/heart.pcd 25 210 pointclouds/heart_rotatedpi4.pcd 25 210 pointclouds/heart.pcd 25 230 pointclouds/heart_rotatedpi4.pcd 25 230 pointclouds/heart.pcd 25 250 pointclouds/heart_rotatedpi4.pcd 25 250 pointclouds/heart.pcd 25 270 pointclouds/heart_rotatedpi4.pcd 25 270 pointclouds/heart.pcd 25 290 pointclouds/heart_rotatedpi4.pcd 25 290 pointclouds/heart.pcd 30 90 pointclouds/heart_rotatedpi4.pcd 30 90 pointclouds/heart.pcd 30 110 pointclouds/heart_rotatedpi4.pcd 30 110 pointclouds/heart.pcd 30 130 pointclouds/heart_rotatedpi4.pcd 30 130 pointclouds/heart.pcd 30 150 pointclouds/heart_rotatedpi4.pcd 30 150 pointclouds/heart.pcd 30 170 pointclouds/heart_rotatedpi4.pcd 30 170 pointclouds/heart.pcd 30 190 pointclouds/heart_rotatedpi4.pcd 30 190 pointclouds/heart.pcd 30 210 pointclouds/heart_rotatedpi4.pcd 30 210 pointclouds/heart.pcd 30 230 pointclouds/heart_rotatedpi4.pcd 30 230 pointclouds/heart.pcd 30 250 pointclouds/heart_rotatedpi4.pcd 30 250 pointclouds/heart.pcd 30 270 pointclouds/heart_rotatedpi4.pcd 30 270 pointclouds/heart.pcd 30 290 pointclouds/heart_rotatedpi4.pcd 30 290 pointclouds/heart.pcd 35 90 pointclouds/heart_rotatedpi4.pcd 35 90 pointclouds/heart.pcd 35 110 pointclouds/heart_rotatedpi4.pcd 35 110 pointclouds/heart.pcd 35 130 pointclouds/heart_rotatedpi4.pcd 35 130 pointclouds/heart.pcd 35 150 pointclouds/heart_rotatedpi4.pcd 35 150 pointclouds/heart.pcd 35 170 pointclouds/heart_rotatedpi4.pcd 35 170 pointclouds/heart.pcd 35 190 pointclouds/heart_rotatedpi4.pcd 35 190 pointclouds/heart.pcd 35 210 pointclouds/heart_rotatedpi4.pcd 35 210 pointclouds/heart.pcd 35 230 pointclouds/heart_rotatedpi4.pcd 35 230 pointclouds/heart.pcd 35 250 pointclouds/heart_rotatedpi4.pcd 35 250 pointclouds/heart.pcd 35 270 pointclouds/heart_rotatedpi4.pcd 35 270 pointclouds/heart.pcd 35 290 pointclouds/heart_rotatedpi4.pcd 35 290 pointclouds/heart.pcd 40 90 pointclouds/heart_rotatedpi4.pcd 40 90 pointclouds/heart.pcd 40 110 pointclouds/heart_rotatedpi4.pcd 40 110 pointclouds/heart.pcd 40 130 pointclouds/heart_rotatedpi4.pcd 40 130 pointclouds/heart.pcd 40 150 pointclouds/heart_rotatedpi4.pcd 40 150 pointclouds/heart.pcd 40 170 pointclouds/heart_rotatedpi4.pcd 40 170 pointclouds/heart.pcd 40 190 pointclouds/heart_rotatedpi4.pcd 40 190 pointclouds/heart.pcd 40 210 pointclouds/heart_rotatedpi4.pcd 40 210 pointclouds/heart.pcd 40 230 pointclouds/heart_rotatedpi4.pcd 40 230 pointclouds/heart.pcd 40 250 pointclouds/heart_rotatedpi4.pcd 40 250 pointclouds/heart.pcd 40 270 pointclouds/heart_rotatedpi4.pcd 40 270 pointclouds/heart.pcd 40 290 pointclouds/heart_rotatedpi4.pcd 40 290'
    with open('computealltransforms.sh', 'w') as outf_transforms:
        for v,s in product(voxels, seeds):
            print(
                ' '.join([
                    computetransform_cmd(
                        'matched_pointclouds/{}.pcd_src_v{}s{}v{}s{}.csv'.format(name, v,s,v,s),
                        'matched_pointclouds/{}_rotatedpi4.pcd_dst_v{}s{}v{}s{}.csv'.format(name, v,s,v,s)
                    ),
                    '> transforms/v{}s{}v{}s{}.txt'.format(v,s,v,s)
                ]),
                file=outf_transforms)

    with open('computealldistances.sh', 'w') as outf_distances:
        distancefilename = 'distances.txt'
        for v,s in product(voxels, seeds):
            params = 'v{}s{}v{}s{}'.format(v,s,v,s)
            print('echo -n "{} " >> {}'.format(params, distancefilename), file=outf_distances)
            print(
                ' '.join([
                    '../../scripts_python/distance_matched_pointclouds_csv.py transforms/{}.txt matched_pointclouds/{}.pcd_src_{}.csv matched_pointclouds/{}_rotatedpi4.pcd_dst_{}.csv'.format(params, name, params, name, params),
                    '>> {}'.format(distancefilename)
                ]),
                file=outf_distances
            )

if __name__=='__main__':
    main(sys.argv)
