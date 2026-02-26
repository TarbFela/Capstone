import os
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np
from scipy.optimize import curve_fit

def expfunc(x, a, b, c):
    return a * np.exp(-b * x) + c

import tc
mytc = tc.Thermocouple()

fig,ax = plt.subplots(2,1,sharex=True);

fps = [fp for fp in os.listdir("260210_sweep_data")]
fps.sort()
i = 0;
for fp in fps:
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
    #Vout =  -214.2785663 * Vdiff + 2.908868568
    tc_data = data[2]
    tc_data = (tc_data - 2.9089)/(-214.28)
    vout_data = -1 * data[1]
    time_data = data[0]

    # Let's isolate data from when we're actually applying power
    # Vout should be big and should have been big for a little bit (or, on the other end, *will* be big for a little bit)
    tc_data = tc_data[abs(vout_data) > 0.1]
    time_data = time_data[abs(vout_data) > 0.1]
    vout_data = vout_data[abs(vout_data) > 0.1]
    # now that we've gotten rid of dead time, let's trim both sides down...
    tc_data = tc_data[10:-10]
    time_data = time_data[10:-10]
    vout_data = vout_data[10:-10]

    # move everything to the front...
    time_data += -time_data[0]

    # now let's get a best-fit?
    popt, pcov = curve_fit(expfunc, time_data, tc_data)
    tc_fit_data = [np.arange(0,500,1),expfunc(np.arange(0,500,1), *popt)]
    ax[1].plot(tc_fit_data[0],tc_fit_data[1],'r--',label="fit " + data_label)

    ax[0].plot(time_data,vout_data,linestyle='-',color="black", label= data_label)
    ax[1].plot(time_data,tc_data,linestyle='-',color="black", label= data_label)
    ax[0].annotate(
        data_label,
        xy=(time_data[-1], vout_data[-1]),
        xytext=(105, 0.2 + 0.15 *i), # Offset slightly to the right
        arrowprops=dict(arrowstyle="-", color='gray'),
        ha='left',
        va='center',
        color='gray',
        fontsize=8
    )
    ax[1].annotate(
        data_label,
        xy=(tc_fit_data[0][-1], tc_fit_data[1][-1]),
        xytext=(550, 0.00025+ 0.0017*i), # Offset slightly to the right
        arrowprops=dict(arrowstyle="-", color='gray'),
        ha='left',
        va='center',
        color='gray',
        fontsize=8
    )
    i += 1

for a in ax:
    #a.legend()
    a.grid(True)

ax[0].set_xlabel("Time (s)")
ax[1].set_xlabel("Time (s)")
ax[0].set_ylabel("Applied Voltage (V)")
ax[1].set_ylabel("Thermocouple Voltage (V)")
ax[0].set_title("Thermocouple Voltages Over Time and Applied Current")

plt.show()
