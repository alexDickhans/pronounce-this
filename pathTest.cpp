
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <graphics.h>

#include <chrono>

#define SIM 0

#include "pathPlanner/utils/bezierSegment.hpp"
#include "include/pathPlanner/utils/point.hpp"
#include "include/pathPlanner/utils/vector.hpp"
#include "velocityProfile/velocityProfile.hpp"
#include "velocityProfile/sinusoidalVelocityProfile.hpp"
#include "utils/utils.hpp"

using namespace PathPlanner;

#define multiplier 3.0

#define lookahead 8

#define starting_point_random 1

// ANCHOR Printing variables

#define PRINT_LIVE true
#define GRAPH true

#define FIELD_WIDTH 140.6

void printRobot(Point point, Angle orientation) {
//	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 15);
	std::cout << orientation.Convert(degree) << std::endl;
	line(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, point.getY().Convert(inch) * multiplier + cos(orientation) * 40, point.getX().Convert(inch) * multiplier + sin(orientation) * 40);
}

void printRobotWithLookahead(Point point) {
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 1);
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, lookahead * multiplier);
}

void printPath(BezierSegment path) {
	line(path.getA().getY().Convert(inch) * multiplier, path.getA().getX().Convert(inch) * multiplier,
		 path.getB().getY().Convert(inch) * multiplier, path.getB().getX().Convert(inch) * multiplier);
	line(path.getD().getY().Convert(inch) * multiplier, path.getD().getX().Convert(inch) * multiplier,
		 path.getC().getY().Convert(inch) * multiplier, path.getC().getX().Convert(inch) * multiplier);

	for (double i = 1.0/10.0; i < 1.0; i += 1.0/20.0) {
		Point lastPoint = path.evaluate(i-(1.0/20.0));
		Point currentPoint = path.evaluate(i);

		line(lastPoint.getY().Convert(inch) * multiplier, lastPoint.getX().Convert(inch) * multiplier, currentPoint.getY().Convert(inch) * multiplier, currentPoint.getX().Convert(inch) * multiplier);
//		circle(lastPoint.getY().Convert(inch) * multiplier, lastPoint.getX().Convert(inch) * multiplier, 2);
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
	return {resultX * 1_in, resultY * 1_in};
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

void printRing(Point point) {
	circle(point.getY().Convert(inch) * multiplier, point.getX().Convert(inch) * multiplier, 8);
}

void printRing(double x, double y) {
	circle(y * multiplier, x * multiplier, 8);
}

void printMirroredRing(Point point) {
	printRing(point);
	printRing(mirror(point));
}

void printField() {

	printFieldGrid();

	printMirroredTape(1, FIELD_WIDTH/2, FIELD_WIDTH-1, FIELD_WIDTH/2);
	printMirroredTape(FIELD_WIDTH/2, 1, FIELD_WIDTH/2, FIELD_WIDTH-1);
	printMirroredTape(1, FIELD_WIDTH/6, FIELD_WIDTH/6, 1);
	printMirroredTape(FIELD_WIDTH * 5/6, 1, FIELD_WIDTH-1, FIELD_WIDTH/6);

	printMirroredPlatform(FIELD_WIDTH / 3, 1, FIELD_WIDTH * 2.0/ 3.0, FIELD_WIDTH/6.0);

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
	
	setlinestyle(SOLID_LINE, 5, 2);

	std::vector<BezierSegment> paths;

	BezierSegment bezierSegment1 = BezierSegment(Point(12_in, 12_in), Point(12_in, 36_in), Point(36_in, 12_in), Point(36_in, 36_in));
	paths.emplace_back(bezierSegment1);
	BezierSegment bezierSegment2 = BezierSegment(Point(36_in, 36_in), Point(36_in, 60_in), Point(12_in, 36_in), Point(12_in, 60_in));
	paths.emplace_back(bezierSegment2);

	// Print all paths in the vector paths
	for (int i = 0; i < paths.size(); i++) {
		printPath(paths.at(i));
	}

	std::vector<Pronounce::SinusoidalVelocityProfile> profiles;
	Pronounce::ProfileConstraints defaultProfileConstraints = {60_in/second, 200_in/second/second};

	QTime totalTime = 0.0;

	for (int i = 0; i < paths.size(); i++) {
		profiles.emplace_back(paths.at(i).getDistance(), defaultProfileConstraints);
		totalTime += profiles.at(i).getDuration();
	}

	for (QTime currentTime = 0.0; currentTime < totalTime; currentTime += 20_ms) {
		delay(20);
		cleardevice();
		printField();
		for (int i = 0; i < paths.size(); i++) {
			printPath(paths.at(i));
		}

		double index = 0;
		QLength totalDistance = 0.0;
		QTime time = currentTime;

		while (time >= profiles.at(index).getDuration()) {
			time -= profiles.at(index).getDuration();
			totalDistance += paths.at(index).getDistance();
			index += 1;
		}

		index += paths.at(index).getTByLength(profiles.at(index).getDistanceByTime(currentTime));

		Point curvature = paths.at(index).evaluate(fmod(index, 1)) + Vector(paths.at(index).getCurvature(fmod(index, 1)).getValue() * 0.1, paths.at(index).getAngle(fmod(index, 1)) - 90_deg).getCartesian();

		std::cout << "Curvature: " << paths.at(index).getCurvature(fmod(index, 1)).Convert(degree/inch) << std::endl;

		circle(curvature.getY().Convert(inch) * multiplier, curvature.getX().Convert(inch) * multiplier, 2);
		printRobot(paths.at(index).evaluate(fmod(index, 1)), paths.at(index).getAngle(fmod(index, 1)));
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