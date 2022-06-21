#! /usr/bin/env python3

from operator import itemgetter
from itertools import groupby
import matplotlib.pyplot as plt
import numpy as np

def read_data(filename):
    V,S,D = [],[],[]
    with open(filename, 'r') as f:
        for line in f:
            params, d = line.split(' ')
            d = float(d)
            v, s = map(int, params.split('v')[1].split('s'))
            D.append(d)
            V.append(v)
            S.append(s)
    return V,S,D

# def plot_data(V,S,D):
#     vmin,vmax = min(V),max(V)
#
#     plt.plot(v,s,d)

def plot_surface(V,S,D):
    array_of_triplets = [list(g) for _,g in groupby(sorted(zip(V,S,D)),key=itemgetter(0))]
    V = np.array([[v for v,_,_ in row] for row in array_of_triplets])
    S = np.array([[s for _,s,_ in row] for row in array_of_triplets])
    D = np.array([[d for _,_,d in row] for row in array_of_triplets])
    fig,ax = plt.subplots(subplot_kw={'projection':'3d'})
    ax.set_xlabel('voxel')
    ax.set_ylabel('seed')
    ax.set_zlabel('d')
    ax.plot_surface(V,S,D)


V,S,D = read_data('distances.txt')
plot_surface(V,S,D)
plt.show()
