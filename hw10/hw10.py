from ulab import numpy as np

arr = [x for x in range(1024)]

for i, t in enumerate(arr):
    arr[i] = np.sin(t) + np.sin(2*t) + np.sin(3*t)

fft = np.fft(arr)

