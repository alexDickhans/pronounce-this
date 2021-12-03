
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

#define lookahead 7

#define starting_point_random 1

#define PRINT_LIVE false
#define GRAPH true

#define FIELD_WIDTH 140.6

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

    printGoal(Point(70.3, 70.3));
    printGoal(Point(35.0, 70.3));
    printGoal(Point(105.7, 70.3));

    setcolor(CYAN);
    printMirroredRing(Point(5.8, 70.3));
    printMirroredRing(Point(11.4, 70.3));
    printMirroredRing(Point(17.4, 70.3));
    printMirroredRing(Point(23.2, 70.3));
    printMirroredRing(Point(46.8, 70.3));
    printMirroredRing(Point(52.6, 70.3));
    printMirroredRing(Point(58.4, 70.3));

    printMirroredRing(Point(23.2, 76.5));
    printMirroredRing(Point(23.2, 82.3));
    printMirroredRing(Point(23.2, 88.1));
    printMirroredRing(Point(23.2, 93.9));

    printMirroredRingGroup(Point(48, 46.8));
    printMirroredRingGroup(Point(70.3, 46.8));

    setcolor(GREEN);
}

int main() {

    // Create path
    std::vector<Path> paths = std::vector<Path>();

    // Right Steal Right
    Path rightHomeToGoalNeutral;

    rightHomeToGoalNeutral.addPoint(105.7, 8);
    rightHomeToGoalNeutral.addPoint(105.7, 70.3);

    paths.emplace_back(rightHomeToGoalNeutral);

    Path rightNeutralToMidNeutral;

    rightNeutralToMidNeutral.addPoint(105.7, 70.3);
    rightNeutralToMidNeutral.addPoint(82.3, 46.8);
    rightNeutralToMidNeutral.addPoint(70.3, 70.3);

    paths.emplace_back(rightNeutralToMidNeutral);

    Path midNeutralToRightAlliance;

    midNeutralToRightAlliance.addPoint(70.3, 70.3);
    midNeutralToRightAlliance.addPoint(129.1, 35);

    paths.emplace_back(midNeutralToRightAlliance);

    Path rightAllianceToRightRing;

    rightAllianceToRightRing.addPoint(117.5, 46.8);
    rightAllianceToRightRing.addPoint(117.5, 70.3);
    rightAllianceToRightRing.addPoint(117.5, 80);
    rightAllianceToRightRing.addPoint(70.3, 35);

    // paths.emplace_back(rightRingToRightHome);
    paths.emplace_back(rightAllianceToRightRing);

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
    Point firstPosition = paths.at(0).getPath().at(0);

    firstPosition.setX(firstPosition.getX() + ((rand() / 1073741823) - 1) * starting_point_random);
    firstPosition.setY(firstPosition.getY() + ((rand() / 1073741823) - 1) * starting_point_random);
    Point robot = firstPosition;
    Vector robotMovement = Vector();

#if GRAPH
    std::vector<Point> robotPositions;
    robotPositions.emplace_back(paths.at(0).getPath().at(0));
#endif

    // Use a running average to estimate momentum
    const int runningAverageLength = 10;

    RunningAverage<runningAverageLength> runningAverageX;
    RunningAverage<runningAverageLength> runningAverageY;

    // Loop through paths
    for (int i = 0; i < paths.size(); i++) {

        std::vector<Point> pathVector = paths.at(i).getPath();
        Path path = paths.at(i);

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
            for (int i = 0; i < paths.size(); i++) {
                printPath(paths.at(i).getPath());
            }

            setcolor(LIGHTGRAY);
            setlinestyle(SOLID_LINE, 5, 2);
            line(robot.getY() * multiplier, robot.getX() * multiplier, lookaheadPoint.getY() * multiplier, lookaheadPoint.getX() * multiplier);
            printRobotWithLookahead(robot);

            delay(5);
#endif // PRINT_LIVE

            robotPositions.emplace_back(robot);
#endif
        }
    }
#if GRAPH

    printField();

    // printRobotWithLookahead(robot);

    setlinestyle(DASHED_LINE, 5, 2);

    // Print all paths in the vector paths
    for (int i = 0; i < paths.size(); i++) {
        printPath(paths.at(i).getPath());
    }

    robotPositions.emplace_back(robot);

    setlinestyle(SOLID_LINE, 5, 2);

    setcolor(LIGHTGRAY);
    printPath(robotPositions);

    delay(500000);
    closegraph();

#endif // GRAPH
    return 0;
}