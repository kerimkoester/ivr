import numpy as np
import matplotlib.pyplot as plt

x0,y0,x1,y1 = np.genfromtxt('data.txt',unpack=True)
plt.plot(x0,y0,label='desired')
plt.plot(x1,y1,label='end-effector')
plt.legend()
plt.show()