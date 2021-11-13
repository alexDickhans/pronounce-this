    
#include<stdio.h>
#include <iostream>
#include<stdlib.h>
#include<graphics.h>

#include "include/utils/vector.hpp"
#include "src/utils/vector.cpp"
#include "include/utils/pointUtil.hpp"
#include "src/utils/pointUtil.cpp"
#include "include/utils/path.hpp"
#include "src/utils/path.cpp"

using namespace Pronounce;

#define xOffset 150
#define yOffset 200
#define multiplier 2

#define lookahead 20

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

        printRobot(lastPoint);
    }
}

int main() {

    Path path = Path();
    path.addPoint(0, 0);
    path.addPoint(100, 50);
    path.addPoint(150, 0);
    path.addPoint(100, -50);
    path.addPoint(0, -100);

#if GRAPH
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    delay(1000);
    printPath(path);
#endif

    Point robot = Point(0, 0);
    Vector robotMovement = Vector();

    std::vector<Point> pathVector = path.getPath();
#if GRAPH
    std::vector<Point> robotPositions;
#endif

    while (pathVector.at(pathVector.size() - 1).distance(robot) > 0.5) {
        Point lookaheadPoint = path.getLookAheadPoint(robot, lookahead);
        robotMovement = Vector(&robot, &lookaheadPoint);
        robotMovement.normalize();
        //robotMovement = robotMovement.scale(5);
        robot += robotMovement.getCartesian();

#if GRAPH
#if PRINT_LIVE
        printRobotWithLookahead(robot);

        for (int i = 0; i < robotPositions.size(); i++) {
            printRobot(robotPositions.at(i));
        }

        printPath(path);

        // delay(10);
        cleardevice();
#endif // PRINT_LIVE

        robotPositions.emplace_back(robot);
#endif
    }
#if GRAPH

    printRobotWithLookahead(robot);

    for (int i = 0; i < robotPositions.size(); i++) {
        printRobot(robotPositions.at(i));
    }

    robotPositions.emplace_back(robot);

    printPath(path);

    delay(500000);
    closegraph();

#endif // GRAPH
    return 0;
}