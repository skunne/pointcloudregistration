import sys          # argv, exit()

def print_usage_and_exit():
    print('SYNOPSIS')
    print()
    print('{} [-h | --help]'.format(sys.argv[0]))
    print('    print this help message and exit')
    print()
    print('{} ifile ofile [xmin ymin zmin] xmax ymax zmax'.format(sys.argv[0]))
    print('    1) read .pcd file ifile')
    print('    2) remove points in the right parallelelakkzefkopfpiped delimited by the arguments')
    print('    3) write resulting point cloud to file ofile in pcd format')
    print('    Parameters xmin, ymin and zmin default to 0')
    print()
    sys.exit()

def get_args():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']):
        print_usage_and_exit()
    elif (len(sys.argv) not in [6,9]):
        print_usage_and_exit()
    elif (len(sys.argv) == 6):
        infilename = sys.argv[1]
        outfilename = sys.argv[2]
        xmin,ymin,zmin = 0,0,0
        try:
            xmax = float(sys.argv[3])
            ymax = float(sys.argv[4])
            zmax = float(sys.argv[5])
        except ValueError:
            print_usage_and_exit()
    elif (len(sys.argv) == 9):
        infilename = sys.argv[1]
        outfilename = sys.argv[2]
        try:
            xmin = float(sys.argv[3])
            ymin = float(sys.argv[4])
            zmin = float(sys.argv[5])
            xmax = float(sys.argv[6])
            ymax = float(sys.argv[7])
            zmax = float(sys.argv[8])
        except ValueError:
            print_usage_and_exit()
    return (infilename, outfilename, xmin,ymin,zmin,xmax,ymax,zmax)

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

def dont_kill_it(x,y,z, xmin,ymin,zmin,xmax,ymax,zmax):
    return (
        x < xmin or x > xmax or
        y < ymin or y > ymax or
        z < zmin or z > zmax
    )

def filter_pointcloud(pointcloud, xmin,ymin,zmin,xmax,ymax,zmax):
    return [(x,y,z,rgba) for (x,y,z,rgba) in pointcloud if dont_kill_it(x,y,z, xmin,ymin,zmin,xmax,ymax,zmax)]

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
    (infilename, outfilename, xmin,ymin,zmin,xmax,ymax,zmax) = get_args()
    header, pointcloud = read_file(infilename)
    filtered_pointcloud = filter_pointcloud(pointcloud, xmin,ymin,zmin,xmax,ymax,zmax)
    header = fix_header(header, len(filtered_pointcloud))
    write_file(header, filtered_pointcloud, outfilename)

if __name__ == '__main__':
    main()
