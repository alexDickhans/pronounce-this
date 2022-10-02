
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
// #include "include/utils/runningAverage.hpp"
// #include "src/utils/runningAverage.cpp"
#include "include/utils/utils.hpp"
#include "src/utils/utils.cpp"
#include "include/units/units.hpp"
// #include "include/utils/position.hpp"
// #include "src/utils/position.cpp"
// #include "include/utils/purePursuitProfile.hpp"
// #include "src/utils/purePursuitProfile.cpp"
// #include "include/utils/purePursuitProfileManager.hpp"
// #include "src/utils/purePursuitProfileManager.cpp"
// #include "include/feedbackControllers/pid.hpp"
// #include "src/feedbackControllers/pid.cpp"
// #include "include/chassis/abstractDrivetrain.hpp"
// #include "src/chassis/abstractDrivetrain.cpp"
// #include "include/chassis/abstractTankDrivetrain.hpp"
// #include "src/chassis/abstractTankDrivetrain.cpp"
// #include "include/chassis/simDrivetrain.hpp"
// #include "src/chassis/simDrivetrain.cpp"
// #include "include/chassis/simTankDrivetrain.hpp"
// #include "src/chassis/simTankDrivetrain.cpp"
// #include "include/odometry/odometry.hpp"
// #include "src/odometry/odometry.cpp"
// #include "include/odometry/simOdometry.hpp"
// #include "src/odometry/simOdometry.cpp"
// #include "include/motionControl/purePursuit.hpp"
// #include "src/motionControl/purePursuit.cpp"
// #include "include/motionControl/tankPurePursuit.hpp"
// #include "src/motionControl/tankPurePursuit.cpp"

using namespace Pronounce;

int fps = 60;
double playbackMultiplier = 1;

#define xOffset 30.0
#define yOffset 30.0
#define multiplier 3.0

#define lookahead 8

#define starting_point_random 1

// ANCHOR Printing variables

#define PRINT_LIVE true
#define GRAPH true

#define FIELD_WIDTH 140.6

void printRobot(Point point) {
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 1);
}

void printRobotWithLookahead(Point point) {
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 1);
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, lookahead * multiplier);
}

void printPath(Path path) {

	if (path.getPath().size() < 2) {
		return;
	}

	for (int i = 1; i < path.getPath().size(); i++) {
		Point lastPoint = path.getPath().at(i - 1);
		Point currentPoint = path.getPath().at(i);

		line(lastPoint.getY().Convert(inch) * multiplier, lastPoint.getX().Convert(inch) * multiplier, currentPoint.getY().Convert(inch) * multiplier, currentPoint.getX().Convert(inch) * multiplier);

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

		line(lastPoint.getY().Convert(inch) * multiplier, lastPoint.getX().Convert(inch) * multiplier, currentPoint.getY().Convert(inch) * multiplier, currentPoint.getX().Convert(inch) * multiplier);

		// printRobot(currentPoint);
		// printRobot(lastPoint);
	}
}

double mirror(double x) {
	double result = FIELD_WIDTH - x;
	return result;
}

Point mirror(Point point) {
	QLength beforeX = point.getX();
	QLength beforeY = point.getY();
	double resultX = FIELD_WIDTH - beforeX.Convert(inch);
	double resultY = FIELD_WIDTH - beforeY.Convert(inch);
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
	setlinestyle(SOLID_LINE, 5, 10);
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 6.5 * multiplier);
}

void printMirroredGoal(Point point) {
	setcolor(RED);
	printGoal(point);
	setcolor(BLUE);
	printGoal(mirror(point));
}

void printRing(Point point) {
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 2);
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 4);
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
	double x = point.getX().Convert(inch);
	double y = point.getY().Convert(inch);
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

	printMirroredTape(1, 1, FIELD_WIDTH-1, FIELD_WIDTH-1);

	setcolor(YELLOW);

	setcolor(CYAN);

	setcolor(GREEN);
}

int main() {

	int loopcount = 0;

	/*
	// Sim code
	SimTankDrivetrain drivetrain(15.0, 15.0/fps, 60.0/fps);
	SimOdometry odometry(&drivetrain);

	// Shared code
	TankPurePursuit purePursuit(&drivetrain, &odometry, lookahead);

	double trackWidth = drivetrain.getTrackWidth();

	purePursuit.setSpeed(1.0);
	purePursuit.setTurnPid(new PID(1, 0, 0));
	*/

	int gd = DETECT, gm;
	initgraph(&gd, &gm, NULL);

	setbkcolor(BLACK);
	setcolor(DARKGRAY);

	delay(1000);

	cleardevice();

	printField();

	// printRobotWithLookahead(robot);
	
	setlinestyle(DASHED_LINE, 5, 2);

	std::vector<Path> paths;

	Path smoothPath;
		BezierPath rightNeutralGoalToNearRIghtPlatform("enterFarLeftHomeZoneToNearRightPlatform");

		rightNeutralGoalToNearRIghtPlatform.addPoint(SplinePoint(Point(115_in, 70_in), Vector(25_in, M_PI-M_PI_4)));
		rightNeutralGoalToNearRIghtPlatform.addPoint(SplinePoint(Point(80_in, 60_in), Vector(20_in, M_PI_2)));
		rightNeutralGoalToNearRIghtPlatform.addPoint(SplinePoint(Point(35_in, 60_in), Vector(20_in, M_PI_2)));
		rightNeutralGoalToNearRIghtPlatform.addPoint(SplinePoint(Point(35_in, 45_in), Vector(20_in, -M_PI_2*0.9)));

	smoothPath = rightNeutralGoalToNearRIghtPlatform.getPath(0.1);

	paths.emplace_back(smoothPath);

	// Print all paths in the vector paths
	for (int i = 0; i < paths.size(); i++) {
		printPath(paths.at(i));
	}

	delay(50000000);

	/*
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
	*/
	return 0;
}