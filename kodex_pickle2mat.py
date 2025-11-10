from scipy.io import savemat
import pickle
import numpy as np
import os

with open('demo_door.pickle', 'rb') as f:
    pickle_data = pickle.load(f)
data = []

for dic in pickle_data:
    data.append(dic['lifted_states'])
numpy_data = np.array(data)

X = np.reshape(numpy_data[:, :-1, :], (-1, 546))
Y = np.reshape(numpy_data[:, 1:, :], (-1, 546))

os.makedirs('datasets/kodex/', exist_ok=True)

savemat('datasets/kodex/door.mat', {'X': X, 'Y': Y})

with open('demo_pen.pickle', 'rb') as f:
    pickle_data = pickle.load(f)
data = []

for dic in pickle_data:
    data.append(dic['lifted_states'])
numpy_data = np.array(data)

X = np.reshape(numpy_data[:, :-1, :], (-1, 582))
Y = np.reshape(numpy_data[:, 1:, :], (-1, 582))

savemat('datasets/kodex/pen.mat', {'X': X, 'Y': Y})