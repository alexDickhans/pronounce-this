
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <graphics.h>

#include <chrono>

#define SIM 0

#include "include/utils/vector.hpp"
#include "src/utils/vector.cpp"
#include "include/utils/pointUtil.hpp"
#include "src/utils/pointUtil.cpp"
#include "include/utils/path.hpp"
#include "src/utils/path.cpp"
#include "include/utils/splinePath.hpp"
#include "include/utils/quadraticSplinePath.hpp"
#include "include/utils/runningAverage.hpp"
#include "src/utils/runningAverage.cpp"
#include "include/utils/utils.hpp"
#include "src/utils/utils.cpp"
#include "include/utils/position.hpp"
#include "src/utils/position.cpp"
#include "include/utils/purePursuitProfile.hpp"
#include "src/utils/purePursuitProfile.cpp"
#include "include/utils/purePursuitProfileManager.hpp"
#include "src/utils/purePursuitProfileManager.cpp"
#include "include/feedbackControllers/pid.hpp"
#include "src/feedbackControllers/pid.cpp"
#include "include/chassis/abstractDrivetrain.hpp"
#include "src/chassis/abstractDrivetrain.cpp"
#include "include/chassis/abstractTankDrivetrain.hpp"
#include "src/chassis/abstractTankDrivetrain.cpp"
#include "include/chassis/simDrivetrain.hpp"
#include "src/chassis/simDrivetrain.cpp"
#include "include/chassis/simTankDrivetrain.hpp"
#include "src/chassis/simTankDrivetrain.cpp"
#include "include/odometry/odometry.hpp"
#include "src/odometry/odometry.cpp"
#include "include/odometry/simOdometry.hpp"
#include "src/odometry/simOdometry.cpp"
#include "include/motionControl/purePursuit.hpp"
#include "src/motionControl/purePursuit.cpp"
#include "include/motionControl/tankPurePursuit.hpp"
#include "src/motionControl/tankPurePursuit.cpp"

using namespace Pronounce;

int testPathIndex;
int leftAllianceToRightHomeZoneIndex;
int rightHomeZoneToRightAllianceIndex;
int rightAllianceToRightRingsIndex;
int rightRingsToRightHomeZoneIndex;

// Skills
int leftHomeZoneToLeftNeutralGoalIndex;
int leftNeutralGoalToFarHomeZoneIndex;
int farHomeZoneToMidNeutralGoalIndex;
int midNeutralGoalToPlatformIndex;
int farPlatformToDropOffGoalIndex;
int goalDropOffToFarLeftAllianceIndex;
int farLeftAllianceToLeftRingsIndex;
int leftRingsToGoalDropIndex;
int farPlatformToEnterHomezoneIndex;
int rightHomeZoneToFarRightAllianceGoalIndex;
int farRightAllianceGoalToNearPreloadsIndex;
int nearPlatformAlignIndex;
int leftHomeZoneToPlatformIndex;

int fps = 60;
double playbackMultiplier = 1;

#define xOffset 30
#define yOffset 30
#define multiplier 3

#define lookahead 8

#define starting_point_random 1

// ANCHOR Printing variables

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

void printRobot(Odometry odometry, double trackWidth) {
	trackWidth /= 2;

	Position* point = odometry.getPosition();
	circle(point->getY() * multiplier, point->getX() * multiplier, 1);
	circle(point->getY() * multiplier, point->getX() * multiplier, lookahead * multiplier);

	Vector forwardVector(20, point->getTheta() + M_PI_2);

	Point forwardPoint;
	forwardPoint.operator=(point);
	forwardPoint.add(Point(forwardVector.getCartesian().getX(), forwardVector.getCartesian().getY()));

	line(point->getY() * multiplier, point->getX() * multiplier, forwardPoint.getY() * multiplier, forwardPoint.getX() * multiplier);
}

void printRobot(Odometry odometry, double trackWidth, double curvature) {
	printRobot(odometry, trackWidth);

	setcolor(GREEN);

	double radius = 1.0/curvature;

	if (abs(radius) > 100) {
		return;
	}

	Vector curvatureVector(radius, odometry.getPosition()->getTheta());

	// Make sure the radius is positive
	radius = abs(radius);

	Point curvaturePoint;
	curvaturePoint.operator=(odometry.getPosition());
	curvaturePoint.add(curvatureVector.getCartesian());

	circle(curvaturePoint.getY() * multiplier, curvaturePoint.getX() * multiplier, radius * multiplier);

	setcolor(BLACK);
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

		// printRobot(currentPoint);
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

	int loopcount = 0;

	SimTankDrivetrain drivetrain(15.0, 15.0/fps, 60.0/fps);
	SimOdometry odometry(&drivetrain);

	TankPurePursuit purePursuit(&drivetrain, &odometry, lookahead);

	double trackWidth = drivetrain.getTrackWidth(); 

	purePursuit.setSpeed(1.0);
	purePursuit.setTurnPid(new PID(1, 0, 0));

	// Test path
	/*
	SplinePath testPath = SplinePath();

	testPath.addPoint(40, 40);
	testPath.addPoint(40, 64);
	testPath.addPoint(64, 64);

	testPathIndex = purePursuit.addPath(testPath.getPath(0.1));
	*/

	// Left home zone to left neutral goal
	QuadraticSplinePath leftHomeZoneToLeftNeutralGoal;

	leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(23.2, 11.4), Vector(30, M_PI_4*1.5)));
	leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(48, 46.8), Vector(20, 0)));
	leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(35, 66), Vector(20, 0)));

	int leftHomeZoneToLeftNeutralGoalIndex = purePursuit.addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.01));

	QuadraticSplinePath leftNeutralGoalToFarHomeZone;

	leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(35, 66), Vector(30, -M_PI_4)));
	leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70.3, 93.9), Vector(30, 0)));
	leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70.3, 107), Vector(20, 0)));

	int leftNeutralGoalToFarHomeZoneIndex = purePursuit.addPath(leftNeutralGoalToFarHomeZone.getPath(0.01));

	QuadraticSplinePath farHomeZoneToMidNeutralGoal;

	farHomeZoneToMidNeutralGoal.addPoint(SplinePoint(Point(70.3, 107), Vector(20, 0)));
	farHomeZoneToMidNeutralGoal.addPoint(SplinePoint(Point(70.3, 75), Vector(30, 0)));

	int farHomeZoneToMidNeutralGoalIndex = purePursuit.addPath(farHomeZoneToMidNeutralGoal.getPath(0.01));

	QuadraticSplinePath midNeutralGoalToPlatform;

	midNeutralGoalToPlatform.addPoint(SplinePoint(Point(70.3, 75), Vector(30, 0)));
	midNeutralGoalToPlatform.addPoint(SplinePoint(Point(75, 107), Vector(30, 0)));

	int midNeutralGoalToPlatformIndex = purePursuit.addPath(midNeutralGoalToPlatform.getPath(0.01));

	QuadraticSplinePath farPlatformToDropOffGoal;

	farPlatformToDropOffGoal.addPoint(SplinePoint(Point(65, 107), Vector(5, 0)));
	farPlatformToDropOffGoal.addPoint(SplinePoint(Point(65, 80), Vector(5, 0)));

	int farPlatformToDropOffGoalIndex = purePursuit.addPath(farPlatformToDropOffGoal.getPath(0.01));

	QuadraticSplinePath goalDropOffToFarLeftAlliance;

	goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(65, 80), Vector(15, 0)));
	goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4)));

	int goalDropOffToFarLeftAllianceIndex = purePursuit.addPath(goalDropOffToFarLeftAlliance.getPath(0.01));

	QuadraticSplinePath farLeftAllianceToLeftRings;

	farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4 + M_PI)));
	farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, M_PI)));

	int farLeftAllianceToLeftRingsIndex = purePursuit.addPath(farLeftAllianceToLeftRings.getPath(0.01));

	QuadraticSplinePath leftRingsToGoalDrop;

	leftRingsToGoalDrop.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, 0)));
	leftRingsToGoalDrop.addPoint(SplinePoint(Point(60, 73), Vector(2, -M_PI_4)));
	leftRingsToGoalDrop.addPoint(SplinePoint(Point(75, 107), Vector(10, 0)));

	int leftRingsToGoalDropIndex = purePursuit.addPath(leftRingsToGoalDrop.getPath(0.01));

	QuadraticSplinePath farPlatformToEnterHomezone;

	farPlatformToEnterHomezone.addPoint(SplinePoint(Point(75, 107), Vector(2, 0)));
	farPlatformToEnterHomezone.addPoint(SplinePoint(Point(75, 95), Vector(2, 0)));

	int farPlatformToEnterHomezoneIndex = purePursuit.addPath(farPlatformToEnterHomezone.getPath(0.01));

	QuadraticSplinePath enterHomeZoneToRightHomeZone;

	enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(75, 95), Vector(10, 0)));
	enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, -M_PI_2)));

	int enterHomeZoneToRightHomeZoneIndex = purePursuit.addPath(enterHomeZoneToRightHomeZone.getPath(0.01));

	QuadraticSplinePath rightHomeZoneToFarRightAllianceGoal;

	rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, M_PI_2)));
	rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(103, 125), Vector(10, M_PI_4)));

	int rightHomeZoneToFarRightAllianceGoalIndex = purePursuit.addPath(rightHomeZoneToFarRightAllianceGoal.getPath(0.01));

	QuadraticSplinePath farRightAllianceGoalToNearPreloads;

	farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(103, 125), Vector(40, M_PI_4)));
	farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(129.2, 70.3), Vector(20, -M_PI)));
	farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(117.5, 5), Vector(10, 0)));

	int farRightAllianceGoalToNearPreloadsIndex = purePursuit.addPath(farRightAllianceGoalToNearPreloads.getPath(0.01));

	SplinePath nearPlatformAlign;

	nearPlatformAlign.addPoint(117.5, 5);
	nearPlatformAlign.addPoint(117.5, 11.4);
	nearPlatformAlign.addPoint(130, 11.4);

	int nearPlatformAlignIndex = purePursuit.addPath(nearPlatformAlign.getPath(0.01));

	QuadraticSplinePath leftHomeZoneToPlatform;

	leftHomeZoneToPlatform.addPoint(SplinePoint(Point(130, 11.4), Vector(10, -M_PI_2)));
	leftHomeZoneToPlatform.addPoint(SplinePoint(Point(80, 11.4), Vector(10, -M_PI_2)));

	int leftHomeZoneToPlatformIndex = purePursuit.addPath(leftHomeZoneToPlatform.getPath(0.01));

#if GRAPH
	int gd = DETECT, gm;
	initgraph(&gd, &gm, NULL);

	setbkcolor(BLACK);
	setcolor(DARKGRAY);

	delay(1000);
#endif
	Position startingPosition(23.2, 11.4, M_PI_2);

	drivetrain.reset(&startingPosition);

#if GRAPH
	std::vector<Point> robotPositions;
	robotPositions.emplace_back(startingPosition);
#endif

	purePursuit.setEnabled(true);

	// Loop through paths
	for (int i = 0; i < purePursuit.getPaths().size(); i++) {

		purePursuit.setCurrentPathIndex(i);
		purePursuit.setFollowing(true);

		// Loop until you get to the last point
		while (!purePursuit.isDone(1)) {
			loopcount++;

			odometry.update();
			// drivetrain.driveCurvature(-1.0, (1.0 / 24.0));
			purePursuit.update();
			drivetrain.update();

#if GRAPH
#if PRINT_LIVE
			cleardevice();
			printField();

			setlinestyle(SOLID_LINE, 5, 2);

			printPath(robotPositions);

			robotPositions.emplace_back(Point(odometry.getPosition()->getX(), odometry.getPosition()->getY()));

			setlinestyle(DASHED_LINE, 5, 2);
			for (int i = 0; i < purePursuit.getPaths().size(); i++) {
				printPath(purePursuit.getPath(i).getPath());
			}

			setcolor(LIGHTGRAY);
			setlinestyle(SOLID_LINE, 5, 2);
			line(odometry.getPosition()->getY() * multiplier, odometry.getPosition()->getX() * multiplier, purePursuit.getPointData().lookaheadPoint.getY() * multiplier, purePursuit.getPointData().lookaheadPoint.getX() * multiplier);
			printRobot(odometry, 15, purePursuit.getPointData().curvature);

			delay(1000/fps/playbackMultiplier);
#endif // PRINT_LIVE

			robotPositions.emplace_back(Point(odometry.getPosition()->getX(), odometry.getPosition()->getY()));
#endif

			if (loopcount >= 4*fps * purePursuit.getPaths().size() || (odometry.getPosition()->getX() < 0 || odometry.getPosition()->getX() > 144 || odometry.getPosition()->getY() < 0 || odometry.getPosition()->getY() > 144)) {
				break;
			}
		}
	}
#if GRAPH

	cleardevice();

	printField();

	// printRobotWithLookahead(robot);

	setlinestyle(DASHED_LINE, 5, 2);

	// Print all paths in the vector paths
	for (int i = 0; i < purePursuit.getPaths().size(); i++) {
		printPath(purePursuit.getPaths().at(i).getPath());
	}

	robotPositions.emplace_back(Point(odometry.getPosition()->getX(), odometry.getPosition()->getY()));

	setlinestyle(SOLID_LINE, 5, 2);

	setcolor(LIGHTGRAY);
	printPath(robotPositions);

	line(odometry.getPosition()->getY() * multiplier, odometry.getPosition()->getX() * multiplier, purePursuit.getPointData().lookaheadPoint.getY() * multiplier, purePursuit.getPointData().lookaheadPoint.getX() * multiplier);
	printRobot(odometry, 15, purePursuit.getPointData().curvature);

	// Print loopcount
	std::cout << "Time: " << loopcount/(double) fps << std::endl;

	delay(500000);
	closegraph();

#endif // GRAPH

	return 0;
}