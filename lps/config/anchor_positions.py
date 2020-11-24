import os
import csv
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
csv_file = os.path.join(dir_path, 'measurements.csv')

test_loc = []
distances = []

# Read data from csv file
with open(csv_file,'r') as csvfile:
    csv_data = csv.reader(csvfile, delimiter='\n')
    header = next(csv_data)

    for line in csv_data:
        line = line[0].split(',')
        test_loc.append([float(line[0]), float(line[1]), float(line[2])])

        tmp = []
        for i in np.arange(3,len(line)):
            try:
                tmp.append(float(line[i]))
            except:
                tmp.append(0)
        distances.append(tmp)

# Prepare pseude inverse of A
N_anch = len(distances[0])
N_tests = len(test_loc)
A = np.block([np.ones((N_tests,1)),-2*np.array(test_loc)])

AT = np.transpose(A)
ATA = np.matmul(AT, A)
ATAinv = np.linalg.inv(ATA)
A_psInv = np.matmul(ATAinv, AT)

# Calculate estimates for each anchor
position_estimates = []
for i in range(N_anch):
    b = []
    for j in range(N_tests):
        b.append(distances[j][i]**2 - np.linalg.norm(test_loc[j])**2)

    res = np.matmul(A_psInv, b)
    position_estimates.append([res[1],res[2],res[3]])
    if distances[0][i]==0 and distances[1][i]==0:
        print('[ID{}] - N/A'.format(i))
    else:
        print('[ID{}] - ({:.3f},{:.3f},{:.3f})'.format(i, res[1], res[2], res[3]))

print('Done')
