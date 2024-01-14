#!/usr/bin/env python3

import os

# Qual

os.system("pros v5 rm-all")

elim = True

if elim:
    autons = ["FAR 6 MID RUSH", "FAR 6 BARRIER RUSH", "DISRUPTOR ELIM" , "CLOSE RUSH ELIM", "DISRUPTOR AWP", "SKILLS"]
else:
    autons = ["FAR 6 BALL AWP", "FAR 6 MID RUSH AWP", "FAR 4 AWP" , "CLOSE SAFE AWP", "DISRUPTOR AWP", "CLOSE RUSH AWP", "SKILLS"]

offset = 1

for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i) + "\n")
    if elim:
        f.write("#define ELIM\n")
    f.close()

    print ("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")
