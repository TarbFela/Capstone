import os
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np

import tc
mytc = tc.Thermocouple()


fig,ax = plt.subplots(3,1,sharex=True);

for fp in os.listdir("260204_data"):
    data = []
    if ".csv" not in fp:
        continue
    with open("260204_data/"+fp, "r") as f:
        latch = False
        data_label = fp.rstrip(".csv")
        data_nom_current = float( fp.rstrip(".csv")[-4:-2])
        for line in f:
            if not latch:
                if "Time (s)" in line:
                    latch = True
            else:
                data.append([float(datum) for datum in line.rstrip().split(",")])

    data = np.array(data).T


    ax[0].plot(data[0],data[1], label= data_label)
    ax[1].plot(data[0],data[2], label= data_label)
    ax[2].plot(data[0],abs(data[2]*data_nom_current), label = data_label)

ax[2].set_xlabel("Time (s)")
ax[0].set_ylabel("Voltage (V)")
ax[1].set_ylabel("Voltage (V)")
ax[2].set_ylabel("Power (W)")
ax[0].set_title("Thermocouple Voltage")
ax[1].set_title("Load Voltage")
ax[2].set_title("Load Power")
ax[0].grid(True)
ax[1].grid(True)
ax[2].grid(True)
ax[0].legend()
ax[1].legend()
ax[2].legend()
plt.show()
