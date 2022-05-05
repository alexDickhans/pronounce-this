import os
import time

autons = ["SLeft LAWP", "SMid LAWP", "SMid RAWP", "SRight RAWP", "SRR RAWP", "Right Double Steal", "None", "Skills"]

for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i+1));
    f.close()

    print ("pros mu --slot " + str(i+1) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+1) + " --name \"" + autons[i] + "\"")
