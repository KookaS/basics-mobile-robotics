import numpy as np
import matplotlib.pyplot as plt
import time
from matplotlib.animation import FuncAnimation

x = []
y = []

figure, ax = plt.subplots(figsize=(4, 3))
line, = ax.plot(x, y)
plt.axis([0, 4 * np.pi, -1, 1])


def func_animate(i):
    x = np.linspace(0, 4 * np.pi, 1000)
    y = np.sin(2 * (x - 0.1 * i))

    line.set_data(x, y)

    return line,


ani = FuncAnimation(figure,
                    func_animate,
                    frames=10,
                    interval=50)
k=0
plt.show()
while(k<5):
    k=k+1
    time.sleep(2)
    plt.show()
    print("k=",k)
ani.save(r'animation.gif', fps=10)

plt.show()