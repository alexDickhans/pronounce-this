import matplotlib.pyplot as plt

def main():
    dataKeys = []
    dataValues = []

    totalVelocity = 0

    maxAcceleration = 0;

    userInput = input()

    userInputData = userInput.split(" ")

    dataKeyName = userInputData[0].split(":")[0]
    dataValuesName = userInputData[1].split(":")[0]
    
    lastSpeed = float(userInputData[1].split(":")[1])

    while (userInput != "Done"):
        
        userInputData = userInput.split(" ")

        dataKeys.append(float(userInputData[0].split(":")[1]))
        dataValues.append(float(userInputData[1].split(":")[1]))

        totalVelocity += float(userInputData[1].split(":")[1])*0.01

        maxAcceleration = abs(float(userInputData[1].split(":")[1]) - lastSpeed)*100
        print(maxAcceleration)

        lastSpeed = float(userInputData[1].split(":")[1])

        userInput = input()
        
    print(totalVelocity);

    plt.plot(dataKeys, dataValues)
    plt.xlabel(dataKeyName)
    plt.ylabel(dataValuesName)
    plt.show()

main()
