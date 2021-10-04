import numpy as np

def calculate_std(file_path, axis=1, skip_first_line = True):
    with open(file_path) as reader:
        if skip_first_line:
            reader.readline()
        
        values = list(map(lambda columns: float(columns[axis]), (map(lambda line: line.removesuffix('\n').split(','), reader.readlines()))))

        return np.std(values)


gps_std = calculate_std('./config/log/Graph1.txt')
acc_std = calculate_std('./config/log/Graph2.txt')

print('Quad.GPS.X std: {}'.format(gps_std))
print('Quad.IMU.AX std: {}'.format(acc_std))