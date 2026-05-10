#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os

ls = sorted(os.listdir("logs/"))

for item in ls:
    print(item)

ui = input("Paste the name of the log file you want to graph. Leave blank for most recent.")

if len(ui) == 0:
    path = "logs/" + sorted(ls)[-1]
for item in ls:
    if item in ui:
        path = "logs/" + item

print(path)

data = {}
with open(path) as f:
    hdr = f.readline().strip().split(",")
    for item in hdr:
        data[item] = []
    for line in f:
        row = line.strip().split(",")
        #print(row)
        row = row[:-1]
        for point in zip(row,hdr):
            data[point[1]].append(float(point[0]))



fig,ax = plt.subplots(2,1,sharex=True)
ax[0].plot(data[hdr[0]])
ax[1].plot(data[hdr[1]])
plt.show()

