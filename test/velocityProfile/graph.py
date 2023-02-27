import matplotlib.pyplot as plt

def main():
    dataKeys = []
    dataValues = []

    totalVelocity = 0

    userInput = input()

    userInputData = userInput.split(" ")

    dataKeyName = userInputData[0].split(":")[0]
    dataValuesName = userInputData[1].split(":")[0]
    
    while (userInput != "Done"):
        
        userInputData = userInput.split(" ")

        dataKeys.append(float(userInputData[0].split(":")[1]))
        dataValues.append(float(userInputData[1].split(":")[1]))

        totalVelocity += float(userInputData[1].split(":")[1])*0.01

        userInput = input()

    plt.plot(dataKeys, dataValues)
    plt.xlabel(dataKeyName)
    plt.ylabel(dataValuesName)
    plt.show()

    print(totalVelocity);

main()
