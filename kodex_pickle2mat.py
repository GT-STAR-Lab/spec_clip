from scipy.io import savemat
import pickle
import numpy as np

with open('demo.pickle', 'rb') as f:
    pickle_data = pickle.load(f)
data = []

for dic in pickle_data:
    data.append(dic['lifted_states'])
numpy_data = np.array(data)

X = np.reshape(numpy_data[:, :-1, :], (-1, 546))
Y = np.reshape(numpy_data[:, 1:, :], (-1, 546))

savemat('datasets/kodex/door.mat', {'X': X, 'Y': Y})