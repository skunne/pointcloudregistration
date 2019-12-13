#!/usr/bin/env python3

import pandas as pd
import sys
import statistics

def meanVarFile(filename):
    data = pd.read_csv(filename)
    #print(data.columns.values.tolist())
    #print(data['AngleIn34'])
    means = [data[feature].mean() for feature in data.columns.values.tolist()]
    vars = [data[feature].var() for feature in data.columns.values.tolist()]
    return means, vars

def main(batman, robin):
    #print(filename1)
    m1, v1 = meanVarFile(batman)
    #print(m1)
    #print(v1)
    #print()
    #print(filename2)
    m2, v2 = meanVarFile(robin)
    #print(m2)
    #print(v2)
    #print()
    diff_mean = [abs(x - y) for x,y in zip(m1,m2)]
    diff_var = [abs(x - y) for x,y in zip(v1,v2)]
    #print('Diff in means:')
    #print(diff_mean)
    #print('Diff in vars:')
    #print(diff_var)
    print()
    m_avg = statistics.mean(diff_mean)
    m_var = statistics.variance(diff_mean)
    v_avg = statistics.mean(diff_var)
    v_var = statistics.variance(diff_var)
    print('Average diff in mean:  ', m_avg)
    print('Variance diff in mean: ', m_var)
    print('Average diff in var:   ', v_avg)
    print('Variance diff in var:  ', v_var)

main(sys.argv[1], sys.argv[2])
