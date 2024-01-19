#!/usr/bin/env python3

import os
import sys
import time

# Qual

# os.system("pros v5 rm-all")

autons = [
    "FAR 6 MID ELIM",
    "FAR 5 MID AWP",
    "CLOSE AWP DISRUPTOR",
    "CLOSE ELIM DISRUPTOR",
    "CLOSE SAFE AWP",
    "CLOSE RUSH AWP",
    "CLOSE RUSH ELIM",
    "SKILLS"]

offset = 1

if len(sys.argv) > 1:


for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i) + "\n")
    f.close()

    time.sleep(1)

    print ("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")
