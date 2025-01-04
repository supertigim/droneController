import numpy as np 

data = np.loadtxt('spiral8.csv', delimiter=",")

indexes = np.arange(0, len(data), 4)

print(indexes[0], len(data), indexes[-1])
subsample_data = data[indexes]
new_data = np.vstack((subsample_data, subsample_data[:10, :]))
print(new_data.shape)
np.savetxt('spiral86.csv', new_data, delimiter=",")



