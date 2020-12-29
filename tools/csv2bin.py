import numpy as np

tmp = np.loadtxt('2020-11-29-11-19.csv', dtype=np.str, delimiter=",")

data = tmp[1:].astype(np.float32)
print(data.shape, type(data))

data = np.concatenate((data[:, 0:3], np.expand_dims(data[:, 6], axis=-1)), axis=-1)
print(data.shape)
#print(data.dytpe)
data.tofile('./test32.bin')