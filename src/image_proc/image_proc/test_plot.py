import numpy as np
import matplotlib.pyplot as plt


for i in range(100):
    data = np.array([i,i,i,i,i])
    plt.clf()
    plt.plot(data)
    plt.axis([0, 10, 0, 100])
    plt.show(block=False)
    plt.pause(0.001)