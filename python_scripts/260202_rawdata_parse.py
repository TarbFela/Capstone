import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np

def simple_moving_average_convolve(data, window_size):
    weights = np.ones(window_size) / window_size
    sma = np.convolve(data, weights, mode='valid')
    return sma

tsns_data = []
isns_data = []

pageline_i = 0
with open("260202_rawdata.txt","r") as f:
    for line in f:
        #print(line.rstrip(), end="\t\t")
        if "Page" in line:
            pageline_i = 8
        elif(pageline_i > 4):
            for item in line.rstrip().split(" "):
                isns_data.append(int(item,16))
            pageline_i += -1
        elif(pageline_i > 0):
            for item in line.rstrip().split(" "):
                tsns_data.append(int(item,16))
            pageline_i += -1
        #print()

tsns_data = np.array(tsns_data)
# best fit line
# Vin =  -214.2785663 * Vdiff + 2.908868568
tsns_data = tsns_data*3.3/4095
tsns_data += -2.908868568
tsns_data *= 1/(-214.2785663)

tsns_slope = tsns_data[1:] - tsns_data[:-1]
tsns_slope = tsns_slope[abs(tsns_slope)<0.0002]
tsns_slope = simple_moving_average_convolve(tsns_slope,200)

# assuming about 1mv/ºC and 93.75Hz
tsns_slope *= 93.75 * 1000

fig, ax = plt.subplots(3, 1, sharex=True)
ax[0].plot(isns_data)
ax[0].set_title("Current Shunt")
ax[0].set_xlabel("sample")
ax[0].set_ylabel("ADC Value")
ax[1].plot(tsns_data)
ax[1].set_title("Thermocouple")
ax[1].set_xlabel("sample")
ax[1].set_ylabel("ADC Value")
ax[2].plot(tsns_slope)
ax[2].set_title("Temperature Ramp Rate (assuming 1mV-->1ºC and 93.75sps) (moving average = 150 samples)")
ax[2].set_xlabel("sample")
ax[2].set_ylabel("TC slope")
plt.show()
