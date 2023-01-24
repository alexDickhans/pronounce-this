import os
import time

autons = ["FULL AWP", "CLOSE 5 DISC", "RIGHT 5 DISC", "RIGHT 8 DISC", "SKILlS"]

offset = 1

for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i));
    f.close()

    print ("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")
