
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

#define xOffset 0
#define yOffset 0
#define multiplier 3

#define lookahead 10

#define starting_point_random 1

#define PRINT_LIVE true
#define GRAPH true

#define FIELD_WIDTH 144
#define TILE_WIDTH 23.2

void printRobot(Point point) {
    circle(point.getY() * multiplier, point.getX() * multiplier, 1);
}

void printRobotWithLookahead(Point point) {
    circle(point.getY() * multiplier, point.getX() * multiplier, 1);
    circle(point.getY() * multiplier, point.getX() * multiplier, lookahead * multiplier);
}

void printPath(Path path) {

    if (path.getPath().size() < 2) {
        return;
    }

    for (int i = 1; i < path.getPath().size(); i++) {
        Point lastPoint = path.getPath().at(i - 1);
        Point currentPoint = path.getPath().at(i);

        line(lastPoint.getY() * multiplier, lastPoint.getX() * multiplier, currentPoint.getY() * multiplier, currentPoint.getX() * multiplier);

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

        line(lastPoint.getY() * multiplier, lastPoint.getX() * multiplier, currentPoint.getY() * multiplier, currentPoint.getX() * multiplier);

        // printRobot(lastPoint);
    }
}

double mirror(double x) {
    double result = FIELD_WIDTH - x;
    return result;
}

Point mirror(Point point) {
    double beforeX = point.getX();
    double beforeY = point.getY();
    double resultX = FIELD_WIDTH - beforeX;
    double resultY = FIELD_WIDTH - beforeY;
    return Point(resultX, resultY);
}

void printFieldGrid() {
    setcolor(WHITE);
    setlinestyle(SOLID_LINE, 5, 2);

    rectangle(0, 0, FIELD_WIDTH * multiplier, FIELD_WIDTH * multiplier);

    setcolor(DARKGRAY);
    setlinestyle(SOLID_LINE, 5, 2);
    for (double i = 0; i < FIELD_WIDTH; i += FIELD_WIDTH / 6.0) {
        line(i * multiplier, 0, i * multiplier, FIELD_WIDTH * multiplier);
    }
    for (double i = 0; i < FIELD_WIDTH; i += FIELD_WIDTH / 6.0) {
        line(0, i * multiplier, FIELD_WIDTH * multiplier, i * multiplier);
    }
}

void printTape(double x1, double y1, double x2, double y2) {
    setcolor(WHITE);
    setlinestyle(SOLID_LINE, 5, 2);
    line(y1 * multiplier, x1 * multiplier, y2 * multiplier, x2 * multiplier);
}

void printMirroredTape(double x1, double y1, double x2, double y2) {
    printTape(x1, y1, x2, y2);
    printTape(mirror(x1), mirror(y1), mirror(x2), mirror(y2));
}

void printPlatform(double x1, double y1, double x2, double y2) {
    rectangle(y1 * multiplier, x1 * multiplier, y2 * multiplier, x2 * multiplier);
}

void printMirroredPlatform(double x1, double y1, double x2, double y2) {
    setcolor(RED);
    printPlatform(x1, y1, x2, y2);
    setcolor(BLUE);
    printPlatform(mirror(x1), mirror(y1), mirror(x2), mirror(y2));
}

void printGoal(Point point) {
    setlinestyle(SOLID_LINE, 5, 2);
    circle(point.getY() * multiplier, point.getX() * multiplier, 6.5 * multiplier);
}

void printMirroredGoal(Point point) {
    setcolor(RED);
    printGoal(point);
    setcolor(BLUE);
    printGoal(mirror(point));
}

void printRing(Point point) {
    circle(point.getY() * multiplier, point.getX() * multiplier, 2);
    circle(point.getY() * multiplier, point.getX() * multiplier, 4);
}

void printRing(double x, double y) {
    circle(y * multiplier, x * multiplier, 2);
    circle(y * multiplier, x * multiplier, 4);
}

void printMirroredRing(Point point) {
    printRing(point);
    printRing(mirror(point));
}

void printRingGroup(Point point) {
    double x = point.getX();
    double y = point.getY();
    printRing(point);
    printRing(x + 4, y);
    printRing(x - 4, y);
    printRing(x, y + 4);
    printRing(x, y - 4);
}

void printMirroredRingGroup(Point point) {
    printRingGroup(point);
    printRingGroup(mirror(point));
}

void printField() {

    printFieldGrid();

    printMirroredTape(0, 46.8, FIELD_WIDTH, 46.8);
    printMirroredTape(FIELD_WIDTH, 24, FIELD_WIDTH - 24, 48);

    printMirroredPlatform(46, 0, 96, 24);

    printMirroredGoal(Point(40.9, 11.4));
    printMirroredGoal(Point(129.2, 35.0));

    setcolor(YELLOW);

    printGoal(Point(70.3, 72));
    printGoal(Point(35.0, 72));
    printGoal(Point(105.7, 72));

    setcolor(CYAN);
    printMirroredRing(Point(5.8, 72));
    printMirroredRing(Point(11.4, 72));
    printMirroredRing(Point(17.4, 72));
    printMirroredRing(Point(23.2, 72));
    printMirroredRing(Point(46.8, 72));
    printMirroredRing(Point(52.6, 72));
    printMirroredRing(Point(58.4, 72));

    printMirroredRing(Point(24, 76.5));
    printMirroredRing(Point(24, 82.3));
    printMirroredRing(Point(24, 88.1));
    printMirroredRing(Point(24, 93.9));

    printMirroredRingGroup(Point(48, 46.8));
    printMirroredRingGroup(Point(72, 46.8));

    setcolor(GREEN);
}

int main() {

    // Create path
    Path path = Path();
    path.addPoint(105.7, 13.5);
    path.addPoint(105.7, 30);
    path.addPoint(88, 30);
    path.addPoint(64, 53);
    path.addPoint(58, 35);
    path.addPoint(23, 25);
    path.addPoint(14, 20);
    path.addPoint(34, 11.45);

    srand(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

#if GRAPH
    int gd = DETECT, gm;
    initgraph(&gd, &gm, NULL);

    setbkcolor(BLACK);
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
    const int runningAverageLength = 10;

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
        cleardevice();
        printField();

        setlinestyle(SOLID_LINE, 5, 2);

        printPath(robotPositions);

        robotPositions.emplace_back(robot);

        setlinestyle(DASHED_LINE, 5, 2);
        printPath(path);

        printRobotWithLookahead(robot);

        delay(5);
#endif // PRINT_LIVE

        robotPositions.emplace_back(robot);
#endif
    }
#if GRAPH

    printField();

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