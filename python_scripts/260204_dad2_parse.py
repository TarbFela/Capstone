import os
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np

fig,ax = plt.subplots(2,1,sharex=True);

for fp in os.listdir("260204_data"):
    data = []
    if ".csv" not in fp:
        continue
    with open("260204_data/"+fp, "r") as f:
        latch = False
        data_label = fp.rstrip(".csv")
        for line in f:
            if not latch:
                if "Time (s)" in line:
                    latch = True
            else:
                data.append([float(datum) for datum in line.rstrip().split(",")])

    data = np.array(data).T

    ax[0].plot(data[0],data[1], label= data_label)
    ax[1].plot(data[0],data[2], label= data_label)

plt.legend()
plt.show()