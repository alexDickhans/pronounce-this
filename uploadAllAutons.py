import os

autons = ["FAR 6 BALL AWP", "FAR 6 RUSH AWP", "FAR 6 RUSH ELIM" , "CLOSE SAFE AWP", "DISRUPTOR AWP", "DISRUPTOR ELIM", "SKILLS"]

offset = 1

for i in range(len(autons)):
    f = open("include/auton.h", "w")
    f.write("#define AUTON " + str(i));
    f.close()

    print ("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")

    os.system("pros mu --slot " + str(i+offset) + " --name \"" + autons[i] + "\"")
