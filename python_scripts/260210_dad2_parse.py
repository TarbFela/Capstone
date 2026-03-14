import os
import matplotlib.pyplot as plt
import pandas as pd
import csv
import numpy as np
from scipy.optimize import curve_fit

def expfunc(x, a, b, c):
    return a * np.exp(-b * x) + c

# from https://www.keysight.com/us/en/assets/9022-00195/miscellaneous/5306OSKR-MXD-5501-040107_2.htm
# 0-500ºC coeffs
c0 = 0
c1 = 2.508355 * 10**1
c2 = 7.860106 * 10**-2
c3 = -2.503131 * 10**-1
c4 = 8.315270 * 10**-2
c5 = -1.228034 * 10**-2
c6 = 9.804036 * 10**-4
c7 = -4.413030 * 10**-5
c8 = 1.057734 * 10**-6
c9 = -1.052755 * 10**-8

V_TC_20C = 0.000798
def V_to_T(V):
    V = V - V_TC_20C
    T = c0 + c1*V + c2*(V**2) + c3*(V**3) + c4*(V**4) + c5*(V**5) + c6*(V**6) + c7*(V**7) + c8*(V**8) + c9*(V**9)
    return T

def simple_moving_average_convolve(data, window_size):
    weights = np.ones(window_size) / window_size
    sma = np.convolve(data, weights, mode='valid')
    return sma

fig,ax = plt.subplots(3,1,sharex=True);

colors = ['orangered','orange','gold','forestgreen','dodgerblue','turquoise','violet','darkorchid','darkmagenta','palevioletred']

fps = [fp for fp in os.listdir()]
fps.sort()
i = 0;
for fp in fps:
    data = []
    if ".csv" not in fp or "COOL" in fp:
        continue
    with open(fp, "r") as f:
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

    # perform unit conversions, etc
    current = float(fp.rstrip("A.csv"))
    print(current)
    power_data = abs(vout_data) * current
    tc_data = np.array([V_to_T(1000*v) for v in tc_data])
    tc_data_smoothed = simple_moving_average_convolve(tc_data,400)
    rr_data = [
        (tc_data_smoothed[i+1] - tc_data_smoothed[i])/(time_data[i+1]-time_data[i])
        for i, _ in enumerate(tc_data_smoothed[:-1])
    ]

    # now let's get a best-fit?
    #popt, pcov = curve_fit(expfunc, time_data, tc_data)
    #tc_fit_data = [np.arange(0,500,1),expfunc(np.arange(0,500,1), *popt)]
    #ax[1].plot(tc_fit_data[0],tc_fit_data[1],'r--',label="fit " + data_label)

    ax[0].plot(time_data,power_data,linestyle='-', color=colors[i%len(colors)], label= data_label) # ,color="black"
    ax[1].plot(time_data,tc_data,linestyle='-', color=colors[i%len(colors)], label= data_label) # ,color="black"
    #ax[1].plot(time_data[:(len(tc_data_smoothed))],tc_data_smoothed,linestyle='--', color=colors[i%len(colors)], label= data_label) # ,color="blue"
    ax[2].plot(time_data[:(len(tc_data_smoothed)-1)],rr_data,linestyle='-', color=colors[i%len(colors)], label= data_label) # ,color="black"
    # ax[0].annotate(
    #     data_label,
    #     xy=(time_data[-1], vout_data[-1]),
    #     xytext=(50, 0.2 + 0.15 *i), # Offset slightly to the right
    #     arrowprops=dict(arrowstyle="-", color='gray'),
    #     ha='left',
    #     va='center',
    #     color='gray',
    #     fontsize=8
    # )
    # ax[1].annotate(
    #     data_label,
    #     xy=(tc_fit_data[0][-1], tc_fit_data[1][-1]),
    #     xytext=(550, 0.00025+ 0.0017*i), # Offset slightly to the right
    #     arrowprops=dict(arrowstyle="-", color='gray'),
    #     ha='left',
    #     va='center',
    #     color='gray',
    #     fontsize=8
    # )
    i += 1

for a in ax:
    #a.legend()
    a.grid(True)
ax[0].legend()
ax[0].set_xlabel("Time (s)")
ax[1].set_xlabel("Time (s)")
ax[2].set_xlabel("Time (s)")
ax[0].set_ylabel("Applied Power (W)")
ax[1].set_ylabel("Thermocouple Signal (ºC)")
ax[2].set_ylabel("Ramp Rate (ºC/s)")
ax[0].set_title("Thermocouple Voltages Over Time and Applied Current")

plt.show()
