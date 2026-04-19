d = {}

with open("MCU Pinout.md") as f:
    for line in f:
        if(len(line)) > 1:
            sides = line.split(":")
            left = sides[0].split(" ")
            if(left[-1]) == '':
                label = left[-2]
            else:
                label = left[-1]
            label = label.replace("•","")
            label = label.replace("E","")
            label = label.replace("|","")
            try:
                label = int(label)
            except:
                label = label
            value = sides[1].strip()

            d[label] = value


keys_num = [key for key in d.keys() if type(key) == int]
keys_num = sorted(keys_num)
keys_other = [key for key in d.keys() if type(key) != int]
keys_other = sorted(keys_other)

with open("MCU Pinout Fixer.txt", "x") as f:
    for key in keys_num:
        f.write(f"GPIO {key}\t\t{d[key]}\n")
    for key in keys_other:
        f.write(f"{key}\t\t{d[key]}\n")