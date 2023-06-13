import numpy as np
import matplotlib.pyplot as plt



listx=np.linspace(0,2,90)
listy=np.loadtxt('/home/fabio/DeformableDemo/Errori_sim/Pinch/Error_em_pinch.txt')
plt.ylabel('EM-Error')
plt.xlabel('Time Step')
plt.title('EM error')
plt.plot(listx,listy)
plt.grid()
plt.show()
