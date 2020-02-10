import sys          # argv, exit()

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} ifile ofile'.format(sys.argv[0]))
    print('    1) read .pcd file <ifile>')
    print('    2) remove every 2 out of 3 points')
    print('    3) write resulting point cloud to file ofile in pcd format')
    #print('    Parameters xmin, ymin and zmin default to 0')
    print()
    sys.exit()

def get_args():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']):
        print_usage_and_exit()
    elif len(sys.argv) > 2:
        infilename = sys.argv[1]
        outfilename = sys.argv[2]
    return (infilename, outfilename)

def read_file(infilename):
    header = []
    pointcloud = []
    with open(infilename, 'r') as f:
        for line in f:
            row = line.split()
            if row[0] in ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']:
                header.append(line)
            else:
                (sx,sy,sz,srgba) = row
                (x,y,z,rgba) = (float(sx), float(sy), float(sz), int(srgba))
                pointcloud.append((x,y,z,rgba))
    return header, pointcloud

def dont_kill_it(i):
    return (
        i % 3 == 0
    )

def filter_pointcloud(pointcloud):
    return [(x,y,z,rgba) for (i,(x,y,z,rgba)) in enumerate(pointcloud) if dont_kill_it(i)]

def fix_header(header, new_nb_points):
    new_header = []
    for line in header:
        row = line.split()
        if row[0] in ['WIDTH', 'POINTS']:
            new_header.append('{} {}\n'.format(row[0], new_nb_points))
        else:
            new_header.append(line)
    return new_header

def write_file(header, pointcloud, outfilename):
    with open(outfilename, 'w') as f:
        for line in header:
            f.write(line)
        for (x,y,z,rgba) in pointcloud:
            f.write("{} {} {} {}\n".format(x, y, z, rgba))

def main():
    (infilename, outfilename) = get_args()
    header, pointcloud = read_file(infilename)
    filtered_pointcloud = filter_pointcloud(pointcloud)
    header = fix_header(header, len(filtered_pointcloud))
    write_file(header, filtered_pointcloud, outfilename)

if __name__ == '__main__':
    main()
