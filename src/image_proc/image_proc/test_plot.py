import numpy as np
import matplotlib.pyplot as plt
import time

plt.ion()
fig = plt.figure()
ax = plt.subplot(1,1,1)
ax.set_xlabel('Time')
ax.set_ylabel('Value')
t = []
y = []
ax.plot( t , y , 'ko-' , markersize = 10 ) # add an empty line to the plot
fig.show() # show the window (figure will be in foreground, but the user may move it to background)
fig.canvas.flush_events()
time.sleep(1)
# for i in range(100):
#     data = np.array([i,i,i,i,i])
#     plt.clf()
#     plt.plot(data)
#     plt.axis([0, 10, 0, 100])
#     plt.show(block=False)
#     plt.pause(0.001)