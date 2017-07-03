#!/usr/bin/env python

from sys import argv
import matplotlib.pyplot as plt
import numpy as np

if len(argv) < 2:
	print "Usage: python angle_comparison.py <data_file>"
	exit(0)

f = open(argv[1], 'r')
raw = f.read()
f.close()

lines = raw.split('\n')
lines = filter(lambda x: len(x), lines)

data = map(lambda x: x.strip().split(","), lines)
data = [ [float(i.strip()) for i in row if len(i)] for row in data]
data = np.asarray(data)

plt.hold(True)
plt.title("X")
plt.plot(data[:, 0], label="DSO")
plt.plot(data[:, 3], label="IMU")
plt.plot(data[:, 6], label="GT")
plt.legend()

plt.figure()
plt.title("Y")
plt.hold(True)
plt.plot(data[:, 1], label="DSO")
plt.plot(data[:, 4], label="IMU")
plt.plot(data[:, 7], label="GT")
plt.legend()

plt.figure()
plt.title("Z")
plt.hold(True)
plt.plot(data[:, 2], label="DSO")
plt.plot(data[:, 5], label="IMU")
plt.plot(data[:, 8], label="GT")
plt.legend()

plt.show()