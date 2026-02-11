import os
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np
from scipy.optimize import curve_fit

import tc
mytc = tc.Thermocouple()

fig,ax = plt.subplots(2,1,sharex=True);

for fp in os.listdir("260210_sweep_data"):
    data = []
    if ".csv" not in fp or "COOL" in fp:
        continue

    with open("260210_sweep_data/"+fp, "r") as f:
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
    #Vout =  -214.2785663 * Vdiff + 2.908868568
    tc_data = data[2]
    tc_data = (tc_data - 2.9089)/(-214.28)
    ax[1].plot(data[0],tc_data, label= data_label)

for a in ax:
    a.legend()
    a.grid(True)

plt.show()
