import sys
import numpy as np

filename = sys.argv[1]

fin = open(filename, 'r')
lines = fin.readlines()
data = [[float(x) for x in line.strip().split()] for line in lines[2:]]
data = np.array(data)

data_refined = []
for i in range(1, len(data)):
    if data[i, 0] > data[i - 1, 0] + 1e-3:
        data_refined.append(data[i])
data_refined = np.array(data_refined)
fout = open(filename[:-4] + '_no_duplicates.tum', 'w')
for line in data_refined:
    fout.write(' '.join(map(str, line)) + '\n')
fout.close()