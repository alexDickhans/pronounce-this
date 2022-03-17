#pragma once

#include "utils/pointUtil.hpp"
#include "utils/path.hpp"
#include "utils/splinePath.hpp"
#include "utils/quadraticSplinePath.hpp"
#include "utils/splinePoint.hpp"
#include "motionControl/purePursuit.hpp"
#include "utils/purePursuitProfile.hpp"
#include "utils/purePursuitProfileManager.hpp"

namespace Pronounce {

	// Test path
	int testPathIndex;

	// Left AWP
	int leftAllianceToRightAllianceIndex = -1;
	int rightRingsToRightHomeZone2Index = -1;
	int rightAllianceToRightRingsIndex = -1;

	// Skills
	int leftHomeZoneToLeftNeutralGoalIndex = -1;
	int leftNeutralGoalToFarHomeZoneIndex = -1;
	int farLeftHomeZoneToFarRightGoalDropOffIndex = -1;

	// Right steal right
	int rightHomeZoneToRightNeutralIndex = -1;
	int rightNeutralToRightAllianceGoalIndex = -1;
	int rightAllianceGoalToRightRingsIndex = -1;
	int rightRingsToRightHomeZoneIndex = -1;

	// Left steal Left
	int leftHomeZoneToLeftNeutralIndex = -1;
	int leftNeutralToLeftAllianceGoalIndex = -1;
	int leftAllianceGoalToMidRingsIndex = -1;
	int leftAllianceToRightHomeZoneIndex = -1;
	int leftNeutralToEnterLeftHomeZoneIndex = -1;
	int enterLeftHomeZoneToMidGoalIndex = -1;
	int midGoalToEnterLeftHomeZoneIndex = -1;
	int midRingsToRightHomeZoneIndex = -1;

	// Legacy 
	int forwardIndex = -1;
	int backwardIndex = -1;

	int wiggleIndex = -1;

	void autoPaths(PurePursuit* purePursuit) {
		printf("Why the unknown error, is it in this function??\n");

		// Default pure pursuit profile
		PurePursuitProfile defaultProfile(new PID(20, 0.0, 2.0), new PID(60.0, 0.0, 5.0), 10.0);
		purePursuit->getPurePursuitProfileManager().setDefaultProfile(defaultProfile);

		//SECTION: Test path

		Path testPath = Path();

		testPath.addPoint(0, 0);
		testPath.addPoint(0, 100);

		testPathIndex = purePursuit->addPath(testPath);

		// !SECTION

		//SECTION Skills

		// Left home zone to left neutral goal
		QuadraticSplinePath leftHomeZoneToLeftNeutralGoal("Left Home Zone to Left Neutral Goal");

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(35, 11.4), Vector(30, M_PI_2)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(46, 46.8), Vector(10, 0)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(40, 63), Vector(10, 0.75)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.05));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone("Left Neutral Goal to Far Home Zone");

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(37, 72), Vector(25, 0.1)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70, 100), Vector(8, 0)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(68, 115), Vector(10, 0)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.05));

		QuadraticSplinePath farLeftHomeZoneToFarRightGoalDropOff("Left Home Zone to far right goal drop off");

		farLeftHomeZoneToFarRightGoalDropOff.addPoint(SplinePoint(Point(60, 102), Vector(0.0, 0.0)));
		farLeftHomeZoneToFarRightGoalDropOff.addPoint(SplinePoint(Point(82, 102), Vector(0.0, 0.0)));

		farLeftHomeZoneToFarRightGoalDropOffIndex = purePursuit->addPath(farLeftHomeZoneToFarRightGoalDropOff.getPath(0.05));

		// !SECTION

		// SECTION Left AWP

		QuadraticSplinePath leftAllianceToRightAlliance("Left alliance to right home zone");

		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(28.0, 11.4), Vector(20, M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(30, 43), Vector(20, -M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(80, 43), Vector(15, -M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(125.0, 35), Vector(15.0, -M_PI_2)));

		leftAllianceToRightAllianceIndex = purePursuit->addPath(leftAllianceToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightRings("Right alliance to right rings");

		rightAllianceToRightRings.addPoint(SplinePoint(Point(125, 35), Vector(5, M_PI_4)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(120, 73), Vector(35, 0)));

		rightAllianceToRightRingsIndex = purePursuit->addPath(rightAllianceToRightRings.getPath(0.1));

		QuadraticSplinePath rightRingsToRightHomeZone2("Right rings to right home zone");

		rightRingsToRightHomeZone2.addPoint(SplinePoint(Point(117.5, 70.3), Vector(15.0, M_PI)));
		rightRingsToRightHomeZone2.addPoint(SplinePoint(Point(117.5, 35), Vector(15.0, M_PI)));

		rightRingsToRightHomeZone2Index = purePursuit->addPath(rightRingsToRightHomeZone2.getPath(0.1));

		// !SECTION

		// SECTION Right Steal Right

		QuadraticSplinePath rightHomeZoneToRightNeutral("Right Home Zone to Right Neutral");

		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(105.7, 16.5), Vector(10, 0)));
		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(105.7, 55), Vector(10, 0)));

		rightHomeZoneToRightNeutralIndex = purePursuit->addPath(rightHomeZoneToRightNeutral.getPath(0.1));

		QuadraticSplinePath rightNeutralToRightAllianceGoal;

		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(100, 58), Vector(5, -M_PI*0.9)));
		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(127.0, 37), Vector(35, -M_PI_2*0.9)));

		rightNeutralToRightAllianceGoalIndex = purePursuit->addPath(rightNeutralToRightAllianceGoal.getPath(0.05));

		SplinePath rightAllianceGoalToRightRings;

		rightAllianceGoalToRightRings.addPoint(123.0, 35);
		rightAllianceGoalToRightRings.addPoint(117.5, 46.8);
		rightAllianceGoalToRightRings.addPoint(117.5, 73);
		rightAllianceGoalToRightRings.addPoint(135, 67);

		rightAllianceGoalToRightRingsIndex = purePursuit->addPath(rightAllianceGoalToRightRings.getPath(0.05));

		Path rightRingsToRightHomeZone;

		rightRingsToRightHomeZone.addPoint(135, 67);
		rightRingsToRightHomeZone.addPoint(117.5, 35);

		rightRingsToRightHomeZoneIndex = purePursuit->addPath(rightRingsToRightHomeZone);

		//!SECTION

		// SECTION Left Steal Left

		QuadraticSplinePath leftHomeZoneToLeftNeutral("Left Home Zone to Left Neutral");

		leftHomeZoneToLeftNeutral.addPoint(SplinePoint(Point(25, 16), Vector(0.0, 0.0)));
		leftHomeZoneToLeftNeutral.addPoint(SplinePoint(Point(34, 56), Vector(0.0, 0.0)));

		leftHomeZoneToLeftNeutralIndex = purePursuit->addPath(leftHomeZoneToLeftNeutral.getPath(0.1));

		QuadraticSplinePath leftNeutralToEnterLeftHomeZone("Left alliance to enter left home zone");

		leftNeutralToEnterLeftHomeZone.addPoint(SplinePoint(Point(34, 57), Vector(0.0, 0.0)));
		leftNeutralToEnterLeftHomeZone.addPoint(SplinePoint(Point(30, 40), Vector(0.0, 0.0)));

		leftNeutralToEnterLeftHomeZoneIndex = purePursuit->addPath(leftNeutralToEnterLeftHomeZone.getPath(0.1));

		QuadraticSplinePath enterLeftHomeZoneToMidGoal("Enter Right Home Zone to Mid Goal");

		enterLeftHomeZoneToMidGoal.addPoint(SplinePoint(Point(30, 40), Vector(10, -M_PI_4)));
		enterLeftHomeZoneToMidGoal.addPoint(SplinePoint(Point(65, 65), Vector(0.0, 0.0)));

		enterLeftHomeZoneToMidGoalIndex = purePursuit->addPath(enterLeftHomeZoneToMidGoal.getPath(0.1));

		QuadraticSplinePath midGoalToEnterLeftHomeZone("Mid Goal to Enter Left Home Zone");

		midGoalToEnterLeftHomeZone.addPoint(SplinePoint(Point(30, 40), Vector(0.0, 0.0)));
		midGoalToEnterLeftHomeZone.addPoint(SplinePoint(Point(65, 65), Vector(0.0, 0.0)));

		midGoalToEnterLeftHomeZoneIndex = purePursuit->addPath(midGoalToEnterLeftHomeZone.getPath(0.1));

		SplinePath leftNeutralToLeftAllianceGoal;

		leftNeutralToLeftAllianceGoal.addPoint(30, 58);
		leftNeutralToLeftAllianceGoal.addPoint(6, 17);
		leftNeutralToLeftAllianceGoal.addPoint(34, 19);

		leftNeutralToLeftAllianceGoalIndex = purePursuit->addPath(leftNeutralToLeftAllianceGoal.getPath(0.01));

		QuadraticSplinePath leftAllianceToRightHomeZone;

		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(30.0, 11.4), Vector(15, M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(30, 46.4), Vector(20, -M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(46.4, 46.4), Vector(15, -M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(80, 46.4), Vector(0, -M_PI_2)));

		leftAllianceToRightHomeZoneIndex = purePursuit->addPath(leftAllianceToRightHomeZone.getPath(0.05));

		QuadraticSplinePath leftAllianceGoalToMidRings;

		leftAllianceGoalToMidRings.addPoint(SplinePoint(Point(30.0, 11.4), Vector(0, M_PI_2)));
		leftAllianceGoalToMidRings.addPoint(SplinePoint(Point(15.2, 35.0), Vector(0, 0.0)));
		leftAllianceGoalToMidRings.addPoint(SplinePoint(Point(46.4, 37), Vector(0, M_PI_2)));
		leftAllianceGoalToMidRings.addPoint(SplinePoint(Point(80, 26), Vector(0, M_PI_2)));

		leftAllianceGoalToMidRingsIndex = purePursuit->addPath(leftAllianceGoalToMidRings.getPath(0.01));

		// !SECTION

		QuadraticSplinePath wiggle;

		wiggle.addPoint(SplinePoint(Point(20, 0), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(10, 20), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(20, 40), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(10, 60), Vector(10.0, 0.0)));

		wiggleIndex = purePursuit->addPath(wiggle.getPath(0.01));

		printf("It isn't: Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
