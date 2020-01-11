import pandas as pd
import numpy as np
data = pd.read_csv('data.txt', sep=", ",header = None)
data.to_csv(r'/home/vivek/Desktop/swarm/swarm_ws/src/arduino/src/voltage_controller/data.csv', index=False)
np.savetxt(r'/home/vivek/Desktop/swarm/swarm_ws/src/arduino/src/voltage_controller/data1.txt', data.values, fmt='%.f')
print(data.shape)
