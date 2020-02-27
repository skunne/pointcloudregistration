import sys      # sys.argv

def write_csv(pcd_in, csv_out):
    csv_out.write("id,dimension_1,dimension_2,dimension_3\n")
    i = 0
    for line in pcd_in:
        row = line.split()
        if row[0] not in ['VERSION', 'FIELDS', 'SIZE', 'TYPE', 'COUNT', 'WIDTH', 'HEIGHT', 'VIEWPOINT', 'POINTS', 'DATA']:
            csv_out.write("{},{},{},{}\n".format(i, float(row[0]), float(row[1]), float(row[2])))

def main():
    if len(sys.argv) == 3:
        with open(sys.argv[1], 'r') as pcd_in:
            with open(sys.argv[2], 'w') as csv_out:
                write_csv(pcd_in, csv_out)
    else:
        print('USAGE:')
        print('  {} pcdfile csvfile'.format(sys.argv[0]))

if __name__=='__main__':
    main()
