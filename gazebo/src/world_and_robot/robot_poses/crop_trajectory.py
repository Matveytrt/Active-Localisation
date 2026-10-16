import sys
import numpy as np

filename = sys.argv[1]
start_ts = float(sys.argv[2])
end_ts = float(sys.argv[3])

fin = open(filename, 'r')
lines = fin.readlines()
data = [[float(x) for x in line.strip().split()] for line in lines[2:]]
data = np.array(data)

start_idx = np.where(data[:, 0] >= start_ts)[0][0]
end_idx = np.where(data[:, 0] <= end_ts)[0][-1]

data = data[start_idx:end_idx + 1]
fout = open(filename[:-4] + '_cropped.tum', 'w')
fout.write(lines[0])
fout.write(lines[1])
for line in data:
    fout.write(' '.join(map(str, line)) + '\n')
fout.close()