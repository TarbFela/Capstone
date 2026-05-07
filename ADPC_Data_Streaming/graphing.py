#!/usr/bin/env python3

import matplotlib.pyplot as plt
import os

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
        path = "logs/" + item


for path in paths:
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



    hdr = hdr[:-1]
    fig,ax = plt.subplots(len(hdr),1,sharex=True)
    fig.suptitle(path)
    for i, ch in enumerate(hdr):
        ax[i].ticklabel_format(style='plain', axis='both')
        ax[i].plot(data[ch])
        #ax[0].set_ylim(0,2**24)
        #ax[1].set_ylim(0,2**24)
        ax[i].set_title(ch)
    plt.show()

