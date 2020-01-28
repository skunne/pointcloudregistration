import matplotlib.pyplot as plt  # scatter, xlabel, ylabel, savefig, show
import sys                       # argv for command-line arguments
import os                        # path.splitext to substitute file extensions

##
##  This test is to be used in conjunction with
##  the cpp test ../tests/frankwolfe/test_metricisgood.cpp
##
##  The function test_metricisgood() in test_metricisgood.cpp will write a text file
##  output/scores_as_function_of_matching_quality.txt
##  consisting of several lines containing space-separated numbers
##  Each line if of the form:
##  c  score1 score2 score3 score4...
##  where c represents a number of transpositions (i,j)
##  and all the scores on the line are the result of the product x D x
##  where x is a permutation matrix obtained by multiplying c random transpositions
##  and D is the similarity matrix of the input pointcloud (given with the metadata_filename argument)
##
##  This python script will read the file output/scores_as_function_of_matching_quality.txt
##  And display a scatterplot (and save it to output/scores_as_function_of_matching_quality.png)
##

def read_data(infilename):
    x = []
    y = []
    with open(infilename, 'r') as f:
        for line in f:
            row = line.split()
            complexity = int(row[0])
            for score_str in row[1:]:
                x.append(complexity)
                y.append(float(score_str))
    return x,y

def plot_data(x,y, outfilename):
    plt.scatter(x,y, s=0.7)
    plt.xlabel('Complexity (distance from identity in number of transpositions)')
    plt.ylabel('Score (xDx)')
    plt.title('Scores xDx for different matrices x and point cloud big1.metadata')
    plt.savefig(outfilename)
    plt.show()

def get_filenames():
    if len(sys.argv) > 1:
        infilename = sys.argv[1]
    else:
        infilename = 'output/scores_as_function_of_matching_quality.txt'
    if len(sys.argv) > 2:
        outfilename = sys.argv[2]
    else:
        outfilename = os.path.splitext(infilename)[0]+'.png'
    return infilename, outfilename

def print_usage():
    print('SYNOPSIS')
    print()
    print('  {} [infilename [outfilename]]'.format(sys.argv[0]))
    print()
    print('    infilename defaults to output/scores_as_function_of_matching_quality.txt')
    print('    outfilename defaults to (substitute .png for .txt in infilename)')
    print()
    print('DESCRIPTION')
    print()
    print('  INPUT file infilename and interpret it as a scatterplot.')
    print('  Each line in infilename must be of the form:')
    print('    c  s1 s2 s3 s4...')
    print('    *  c is an integer representing a number of transpositions (i,j)')
    print('    *  s1 s2 etc. are floating-points representing the scores xDx of permutation matrices x obtained by multiplying c random transpositions (i,j)')
    print()
    print('  OUTPUT file outfilename, a scatterplot for s as a function of c')
    print()

def main():
    if (len(sys.argv) > 1 and sys.argv[1] in ['-h', '--help']):
        print_usage()
    else:
        infilename, outfilename = get_filenames()
        x, y = read_data(infilename)
        plot_data(x,y, outfilename)

if __name__ == '__main__':
    main()
