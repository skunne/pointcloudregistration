import sys
import csv

def convert(csv_in, csv_out):
    reader = csv.reader(csv_in)
    writer = csv.writer(csv_out)
    next(reader)
    for row in reader:
        writer.writerow(row[1:])

def main():
    if len(sys.argv) == 3:
        with open(sys.argv[1], 'r') as csv_in:
            with open(sys.argv[2], 'w') as csv_out:
                convert(csv_in, csv_out)
    else:
        print('USAGE:')
        print('  {} <csvin> <csvout>'.format(sys.argv[0]))

if __name__=='__main__':
    main()
