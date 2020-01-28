#import pandas as pd

#df = pd.read_csv('output/esf_similarity_matrix', sep=' ')
#pd.pivot_table(df, index='A', columns='B', values='C')

oldmat = []
with open('output/esf_similarity_matrix') as oldmatfile:
    for line in oldmatfile:
        oldmat.append([float(w) for w in line.split()])

rowlengths = [len(row) for row in oldmat]
assert (min(rowlengths) == max(rowlengths)), ('Error reading similarity matrix file: non-constant row lengths')

nbrows, nbcols = len(oldmat), rowlengths[0]

if (nbcols < nbrows):
    print('second graph has fewer nodes')
    oldmat = [row + [2]*(nbrows - nbcols) for row in oldmat]
elif (nbcols > nbrows):
    print('first graph has fewer nodes')
    oldmat = oldmat + [[2] * nbcols] * (nbcols - nbrows)

newdim = max(nbrows, nbcols)

print('Node similarity matrix before permutation:')
print(oldmat)
print('old sim matrix dims: ', len(oldmat), '*', rowlengths[0])
print('newdim: ', newdim)

with open('output/matching/big1_rot.match') as graphmatchingfile:
    print('Graph matching file header:')
    print(next(graphmatchingfile))
    permutation = [int(line.split()[-1]) - 1 for line in graphmatchingfile]

invperm = [permutation.index(i) for i in range(len(permutation))]

print(sorted(permutation))
print('len(perm): ', len(permutation))
print('Permutation:')
print(permutation)

print('Inverse permutation:')
print(invperm)

print('Max(perm):    ', max(permutation))
print('Max(invperm): ', max(invperm))

newmat = [
    [oldmat[i][invperm[j]] for j in range(newdim)] for i in range(newdim)
]

print('Reordered similarity matrix:')
#print(newmat)

with open('reordered_similarity_matrix', 'w') as outf:
    print('Writing reordered_similarity_matrix to file...')
    for row in newmat:
        for x in row:
            outf.write(str(x))
            outf.write(' ')
        outf.write('\n')
