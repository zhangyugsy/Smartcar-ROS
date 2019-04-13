#!/usr/bin/python
# -*-coding:utf-8-*-
import matplotlib.pyplot as plt
import numpy as np

tested = np.array([717, 1125, 1340, 1700, 2265, 2640])
real = np.array([866, 1280, 1501, 1870, 2421, 2805])

params = np.polyfit(tested, real, 1)
print params

plt.plot(tested, real)
plt.show()
