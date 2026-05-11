#!/usr/bin/env python3

import matplotlib.pyplot as plt
import matplotlib.dates as mdates

import os
import datetime

ls = sorted(os.listdir("logs/"))

for item in ls:
    print(item)

ui = input("Paste the name of the log file you want to graph. Leave blank for most recent. Provide a number to plot the [n] most recent.")

if len(ui) == 0:
    paths = ["logs/" + sorted(ls)[-1]]
if ui.isnumeric():
    paths = ["logs/" + fn for fn in sorted(ls[-int(ui):])]
for item in ls:
    if item in ui:
        paths = ["logs/" + item]


for path in paths:
    print(path)

    data = {}
    with open(path) as f:
        time_start = datetime.datetime.strptime(f.readline().split(": ", 1)[1].strip(), "%Y-%m-%d %H:%M:%S")
        time_stop = datetime.datetime.strptime(f.readline().split(": ", 1)[1].strip(), "%Y-%m-%d %H:%M:%S")
        hdr = f.readline().strip().split(",")
        for item in hdr:
            data[item] = []
        for line in f:
            row = line.strip().split(",")
            #print(row)
            row
            for point in zip(row,hdr):
                try:
                    data[point[1]].append(float(point[0]))
                except ValueError:
                    continue



    #hdr = hdr[:-1]
    fig,ax = plt.subplots(len(hdr),1,sharex=True)
    fig.suptitle(path)
    for i, ch in enumerate(hdr):
        # ax[i].ticklabel_format(style='plain', axis='both')
        ax[i].xaxis.set_major_formatter(
            mdates.ConciseDateFormatter(mdates.AutoDateLocator())
        )
        n_samples = len(data[ch])
        duration  = (time_stop - time_start).total_seconds()
        timestamps = [time_start + datetime.timedelta(seconds=j * duration / (n_samples - 1))
                      for j in range(n_samples)]
        ax[i].plot(timestamps, data[ch])
        #ax[0].set_ylim(0,2**24)
        #ax[1].set_ylim(0,2**24)
        ax[i].set_title(ch)
    plt.show()

