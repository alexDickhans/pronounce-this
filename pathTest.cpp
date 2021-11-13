    
#include<stdio.h>
#include <iostream>
#include<stdlib.h>
#include<graphics.h>

#include <chrono>

#include "include/utils/vector.hpp"
#include "src/utils/vector.cpp"
#include "include/utils/pointUtil.hpp"
#include "src/utils/pointUtil.cpp"
#include "include/utils/path.hpp"
#include "src/utils/path.cpp"
#include "include/utils/runningAverage.hpp"
#include "src/utils/runningAverage.cpp"

using namespace Pronounce;

#define xOffset 150
#define yOffset 200
#define multiplier 2.5

#define lookahead 50

#define starting_point_random 1

#define PRINT_LIVE true
#define GRAPH true


void printRobot(Point point) {

    circle((point.getX() * multiplier) + xOffset, -(point.getY() * multiplier) + yOffset, 1);
    //circle((point.getX() * multiplier)+xOffset, -(point.getY()* multiplier)+yOffset, 5 * multiplier);
}

void printRobotWithLookahead(Point point) {
    circle((point.getX() * multiplier) + xOffset, -(point.getY() * multiplier) + yOffset, 1);
    circle((point.getX() * multiplier) + xOffset, -(point.getY() * multiplier) + yOffset, lookahead * multiplier);
}

void printPath(Path path) {

    if (path.getPath().size() < 2) {
        return;
    }

    for (int i = 1; i < path.getPath().size(); i++) {
        Point lastPoint = path.getPath().at(i - 1);
        Point currentPoint = path.getPath().at(i);

        line((lastPoint.getX() * multiplier) + xOffset, -(lastPoint.getY() * multiplier) + yOffset, (currentPoint.getX() * multiplier) + xOffset, -(currentPoint.getY() * multiplier) + yOffset);

        // printRobot(lastPoint);
    }
}

void printPath(std::vector<Point> path) {

    if (path.size() < 2) {
        return;
    }

    for (int i = 1; i < path.size(); i++) {
        Point lastPoint = path.at(i - 1);
        Point currentPoint = path.at(i);

        line((lastPoint.getX() * multiplier) + xOffset, -(lastPoint.getY() * multiplier) + yOffset, (currentPoint.getX() * multiplier) + xOffset, -(currentPoint.getY() * multiplier) + yOffset);

        // printRobot(lastPoint);
    }
}

int main() {

    // Create path
    Path path = Path();
    path.addPoint(-50, 0);
    path.addPoint(10, 40);
    path.addPoint(150, 40);

    srand(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

#if GRAPH
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    setbkcolor(WHITE);
    setcolor(DARKGRAY);

    delay(1000);
    //printPath(path);
#endif
    // Set random robot starting position
    Point firstPosition = path.getPath().at(0);
    
    firstPosition.setX(firstPosition.getX() + ((rand() / 1073741823) - 1) * starting_point_random);
    firstPosition.setY(firstPosition.getY() + ((rand() / 1073741823) - 1) * starting_point_random);
    Point robot = firstPosition;
    Vector robotMovement = Vector();

    std::vector<Point> pathVector = path.getPath();
#if GRAPH
    std::vector<Point> robotPositions;
    robotPositions.emplace_back(path.getPath().at(0));
#endif

    // Use a running average to estimate momentum
    const int runningAverageLength = 15;

    RunningAverage<runningAverageLength> runningAverageX;
    RunningAverage<runningAverageLength> runningAverageY;

    // Loop until you get to the last point
    while (pathVector.at(pathVector.size() - 1).distance(robot) > 1) {
        // Get the lookahead point 
        Point lookaheadPoint = path.getLookAheadPoint(robot, lookahead);
        robotMovement = Vector(&robot, &lookaheadPoint);
        if (robotMovement.getMagnitude() > 1) {
            robotMovement.normalize();
        }
        robotMovement = robotMovement.scale(1);
        runningAverageX.add(robotMovement.getCartesian().getX());
        runningAverageY.add(robotMovement.getCartesian().getY());

        // Move robot
        Point runningAveragePosition(runningAverageX.getAverage(), runningAverageY.getAverage());
        robot += runningAveragePosition;

#if GRAPH
#if PRINT_LIVE
        printRobotWithLookahead(robot);

        printPath(robotPositions);

        printPath(path);

        delay(10);
        cleardevice();
#endif // PRINT_LIVE

        robotPositions.emplace_back(robot);
#endif
    }
#if GRAPH

    // printRobotWithLookahead(robot);

    setlinestyle(SOLID_LINE, 5, 2);

    printPath(robotPositions);

    robotPositions.emplace_back(robot);

    setlinestyle(DASHED_LINE, 5, 2);
    printPath(path);

    delay(500000);
    closegraph();

#endif // GRAPH
    return 0;
}