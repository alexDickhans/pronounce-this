import os
import time

autons = ["FULL AWP", "CLOSE 9 DISC", "RIGHT 9 DISC", "SKILlS", "TEST MATCH LOAD", "TEST ROLLER"]

offset = 1

for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i));
    f.close()

    print ("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")
