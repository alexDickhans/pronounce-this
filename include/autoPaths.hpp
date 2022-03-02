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
	int farHomeZoneToMidNeutralGoalIndex = -1;
	int farHomeZoneToEnterFarHomeZoneIndex = -1;
	int midNeutralGoalToPlatformIndex = -1;
	int farPlatformToDropOffGoalIndex = -1;
	int dropOffGoalToRightAllianceIndex = -1;
	int rightAllianceToRightNeutralIndex = -1;
	int rightNeutralToFarPlatformIndex = -1;
	int enterFarHomeZoneToGoalDropOffIndex = -1;
	int goalDropOffToFarPlatformIndex = -1;
	int goalDrop2ToFarPlatformIndex = -1;
	int goalDropOffToFarLeftAllianceIndex = -1;
	int farLeftAllianceToLeftRingsIndex = -1;
	int leftRingsToGoalDropIndex = -1;
	int rightHomeZoneToFarRightAllianceGoalIndex = -1;
	int farRightAllianceGoalToNearPreloadsIndex = -1;
	int nearPlatformAlignIndex = -1;
	int leftHomeZoneToPlatformIndex = -1;
	int enterHomeZoneToRightHomeZoneIndex = -1;
	int farHomeZoneToEnterFarHomeZone2Index = -1;
	int farPlatformToGoalDropOff2Index = -1;
	int farLeftAllianceGoalToRightAllianceGoalIndex = -1;

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

	// Legacy 
	int forwardIndex = -1;
	int backwardIndex = -1;

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

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(23.2, 11.4), Vector(15, M_PI_4 * 1.5)));
		// leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(48, 46.8), Vector(15, 0)));
		// leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(30, 70), Vector(20, M_PI_4*0.75)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(35, 65), Vector(20, 0)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.05));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone("Left Neutral Goal to Far Home Zone");

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(33, 68), Vector(10, 0)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70, 90), Vector(10, 0)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(68, 110), Vector(10, 0)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.05));

		Path farHomeZoneToEnterFarHomeZone("Far Home Zone to Enter Far Home Zone");

		farHomeZoneToEnterFarHomeZone.addPoint(70.3, 116);
		farHomeZoneToEnterFarHomeZone.addPoint(70.3, 100);

		farHomeZoneToEnterFarHomeZoneIndex = purePursuit->addPath(farHomeZoneToEnterFarHomeZone);

		Path farHomeZoneToMidNeutralGoal("Far Home Zone to Mid Neutral Goal");

		farHomeZoneToMidNeutralGoal.addPoint(60.3, 117);
		farHomeZoneToMidNeutralGoal.addPoint(66, 78);

		farHomeZoneToMidNeutralGoalIndex = purePursuit->addPath(farHomeZoneToMidNeutralGoal);

		QuadraticSplinePath midNeutralGoalToPlatform("Mid Neutral Goal to Platform");

		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(70, 75), Vector(30, 0)));
		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(73, 113), Vector(30, 0)));

		midNeutralGoalToPlatformIndex = purePursuit->addPath(midNeutralGoalToPlatform.getPath(0.1));

		QuadraticSplinePath farPlatformToDropOffGoal("Far Platform to Drop Off Goal");

		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(80, 113), Vector(20, M_PI)));
		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(40, 97), Vector(15, M_PI_2)));

		farPlatformToDropOffGoalIndex = purePursuit->addPath(farPlatformToDropOffGoal.getPath(0.05));

		QuadraticSplinePath dropOffGoalToRightAlliance("Drop Off Goal to Right Alliance");

		dropOffGoalToRightAlliance.addPoint(SplinePoint(Point(40, 100), Vector(15, -M_PI_2)));
		dropOffGoalToRightAlliance.addPoint(SplinePoint(Point(70, 70), Vector(15, M_PI)));
		dropOffGoalToRightAlliance.addPoint(SplinePoint(Point(120, 35), Vector(40, -M_PI_2)));

		dropOffGoalToRightAllianceIndex = purePursuit->addPath(dropOffGoalToRightAlliance.getPath(0.05));

		QuadraticSplinePath rightAllianceToRightNeutral("Right Alliance to Right Neutral");

		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(125, 35), Vector(20, M_PI_4)));
		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(101.7, 65), Vector(20, 0)));

		rightAllianceToRightNeutralIndex = purePursuit->addPath(rightAllianceToRightNeutral.getPath(0.05));

		QuadraticSplinePath rightNeutralToFarPlatform("Right Neutral Goal to Far Platform");

		rightNeutralToFarPlatform.addPoint(SplinePoint(Point(105.7, 65), Vector(30, 0)));
		rightNeutralToFarPlatform.addPoint(SplinePoint(Point(70, 113), Vector(20, 0)));

		rightNeutralToFarPlatformIndex = purePursuit->addPath(rightNeutralToFarPlatform.getPath(0.1));

		QuadraticSplinePath enterFarHomeZoneToGoalDropOff("Enter Far Home Zone to Goal Drop Off");

		enterFarHomeZoneToGoalDropOff.addPoint(SplinePoint(Point(70, 100), Vector(10, 0)));
		enterFarHomeZoneToGoalDropOff.addPoint(SplinePoint(Point(50, 95), Vector(15, -M_PI_4)));

		enterFarHomeZoneToGoalDropOffIndex = purePursuit->addPath(enterFarHomeZoneToGoalDropOff.getPath(0.1));

		QuadraticSplinePath goalDropOffToFarPlatform("Goal drop off 1 to far platform");

		goalDropOffToFarPlatform.addPoint(SplinePoint(Point(50, 95), Vector(15, -M_PI_4)));
		goalDropOffToFarPlatform.addPoint(SplinePoint(Point(65, 113), Vector(15, 0)));

		goalDropOffToFarPlatformIndex = purePursuit->addPath(goalDropOffToFarPlatform.getPath(0.1));

		QuadraticSplinePath farPlatformToGoalDropOff2("Far platform to goal drop off 2");

		farPlatformToGoalDropOff2.addPoint(SplinePoint(Point(65, 113), Vector(15, M_PI)));
		farPlatformToGoalDropOff2.addPoint(SplinePoint(Point(65, 110), Vector(15, M_PI)));

		farPlatformToGoalDropOff2Index = purePursuit->addPath(farPlatformToGoalDropOff2.getPath(0.1));

		QuadraticSplinePath goalDropOffToFarLeftAlliance("Goal Drop Off to Far Left Alliance");

		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(60, 105.7), Vector(20, M_PI_2)));
		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(18, 105.7), Vector(20, M_PI_2)));

		goalDropOffToFarLeftAllianceIndex = purePursuit->addPath(goalDropOffToFarLeftAlliance.getPath(0.1));

		QuadraticSplinePath farLeftAllianceGoalToRightAllianceGoal("Far Left Alliance Goal to Right Alliance Goal");

		farLeftAllianceGoalToRightAllianceGoal.addPoint(SplinePoint(Point(18, 105.7), Vector(10, -M_PI_2)));
		farLeftAllianceGoalToRightAllianceGoal.addPoint(SplinePoint(Point(95, 105.7), Vector(40, -M_PI_2)));
		farLeftAllianceGoalToRightAllianceGoal.addPoint(SplinePoint(Point(105, 123), Vector(10, M_PI_4)));

		farLeftAllianceGoalToRightAllianceGoalIndex = purePursuit->addPath(farLeftAllianceGoalToRightAllianceGoal.getPath(0.05));

		QuadraticSplinePath farLeftAllianceToLeftRings("Far Left Alliance to Left Rings");

		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4 + M_PI)));
		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, M_PI)));

		farLeftAllianceToLeftRingsIndex = purePursuit->addPath(farLeftAllianceToLeftRings.getPath(0.05));

		QuadraticSplinePath leftRingsToGoalDrop("Left Rings to Goal Drop");

		leftRingsToGoalDrop.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, 0)));
		leftRingsToGoalDrop.addPoint(SplinePoint(Point(60, 73), Vector(15, -M_PI_4 * 0.5)));

		leftRingsToGoalDropIndex = purePursuit->addPath(leftRingsToGoalDrop.getPath(0.05));

		QuadraticSplinePath goalDrop2ToFarPlatform("Goal drop to far platform");

		goalDrop2ToFarPlatform.addPoint(SplinePoint(Point(60, 73), Vector(15, -M_PI_4 * 0.5)));
		goalDrop2ToFarPlatform.addPoint(SplinePoint(Point(75, 110), Vector(10, 0)));

		goalDrop2ToFarPlatformIndex = purePursuit->addPath(goalDrop2ToFarPlatform.getPath(0.05));

		QuadraticSplinePath enterHomeZoneToRightHomeZone("Enter Home Zone to Right Home Zone");

		enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(70, 95), Vector(10, 0)));
		enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, -M_PI_2)));

		enterHomeZoneToRightHomeZoneIndex = purePursuit->addPath(enterHomeZoneToRightHomeZone.getPath(0.05));

		QuadraticSplinePath rightHomeZoneToFarRightAllianceGoal("Right Home Zone to Far Right Alliance Goal");

		rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, M_PI_2)));
		rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(103, 125), Vector(10, M_PI_4)));

		rightHomeZoneToFarRightAllianceGoalIndex = purePursuit->addPath(rightHomeZoneToFarRightAllianceGoal.getPath(0.05));

		QuadraticSplinePath farRightAllianceGoalToNearPreloads("Far Right Alliance Goal to Near Preloads");

		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(103, 125), Vector(20, M_PI + M_PI_4)));
		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(117.5, 70.3), Vector(50, -M_PI)));
		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(105.7, 11.4), Vector(30, M_PI_2 - 0.1)));

		farRightAllianceGoalToNearPreloadsIndex = purePursuit->addPath(farRightAllianceGoalToNearPreloads.getPath(0.1));

		SplinePath nearPlatformAlign;

		nearPlatformAlign.addPoint(117.5, 5);
		nearPlatformAlign.addPoint(117.5, 11.4);
		nearPlatformAlign.addPoint(130, 11.4);

		nearPlatformAlignIndex = purePursuit->addPath(nearPlatformAlign.getPath(0.05));

		QuadraticSplinePath leftHomeZoneToPlatform("Left Home Zone to Platform");

		leftHomeZoneToPlatform.addPoint(SplinePoint(Point(130, 11.4), Vector(10, -M_PI_2)));
		leftHomeZoneToPlatform.addPoint(SplinePoint(Point(115.5, 11.4), Vector(10, -M_PI_2)));

		leftHomeZoneToPlatformIndex = purePursuit->addPath(leftHomeZoneToPlatform.getPath(0.05));

		Path farHomeZoneToEnterFarHomeZone2("Far Home Zone to Enter Far Home Zone");

		farHomeZoneToEnterFarHomeZone2.addPoint(80, 116);
		farHomeZoneToEnterFarHomeZone2.addPoint(85, 100);

		farHomeZoneToEnterFarHomeZone2Index = purePursuit->addPath(farHomeZoneToEnterFarHomeZone2);

		//!SECTION

		// SECTION Left AWP

		QuadraticSplinePath leftAllianceToRightAlliance("Left alliance to right home zone");

		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(28.0, 11.4), Vector(20, M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(30, 43), Vector(20, -M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(80, 43), Vector(15, -M_PI_2)));
		leftAllianceToRightAlliance.addPoint(SplinePoint(Point(123.0, 33), Vector(15.0, -M_PI_2)));

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

		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(100, 58), Vector(5, M_PI)));
		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(127.0, 41), Vector(30, -M_PI_2*0.9)));

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
		leftHomeZoneToLeftNeutral.addPoint(SplinePoint(Point(34, 57), Vector(0.0, 0.0)));

		leftHomeZoneToLeftNeutralIndex = purePursuit->addPath(leftHomeZoneToLeftNeutral.getPath(0.1));

		QuadraticSplinePath leftNeutralToEnterLeftHomeZone("Left alliance to enter left home zone");

		leftNeutralToEnterLeftHomeZone.addPoint(SplinePoint(Point(34, 57), Vector(0.0, 0.0)));
		leftNeutralToEnterLeftHomeZone.addPoint(SplinePoint(Point(30, 40), Vector(0.0, 0.0)));

		leftNeutralToEnterLeftHomeZoneIndex = purePursuit->addPath(leftNeutralToEnterLeftHomeZone.getPath(0.1));

		QuadraticSplinePath enterLeftHomeZoneToMidGoal("Enter Right Home Zone to Mid Goal");

		enterLeftHomeZoneToMidGoal.addPoint(SplinePoint(Point(30, 40), Vector(0.0, 0.0)));
		enterLeftHomeZoneToMidGoal.addPoint(SplinePoint(Point(67, 67), Vector(0.0, 0.0)));

		enterLeftHomeZoneToMidGoalIndex = purePursuit->addPath(enterLeftHomeZoneToMidGoal.getPath(0.1));

		QuadraticSplinePath midGoalToEnterLeftHomeZone("Mid Goal to Enter Left Home Zone");

		midGoalToEnterLeftHomeZone.addPoint(SplinePoint(Point(30, 40), Vector(0.0, 0.0)));
		midGoalToEnterLeftHomeZone.addPoint(SplinePoint(Point(67, 67), Vector(0.0, 0.0)));

		midGoalToEnterLeftHomeZoneIndex = purePursuit->addPath(midGoalToEnterLeftHomeZone.getPath(0.1));

		SplinePath leftNeutralToLeftAllianceGoal;

		leftNeutralToLeftAllianceGoal.addPoint(30, 58);
		leftNeutralToLeftAllianceGoal.addPoint(6, 17);
		leftNeutralToLeftAllianceGoal.addPoint(32, 17);

		leftNeutralToLeftAllianceGoalIndex = purePursuit->addPath(leftNeutralToLeftAllianceGoal.getPath(0.01));

		QuadraticSplinePath leftAllianceToRightHomeZone;

		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(30.0, 11.4), Vector(20, M_PI_2)));
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

		Path forward;

		forward.addPoint(0, 16);
		forward.addPoint(0, 60);

		forwardIndex = purePursuit->addPath(forward);

		Path backward;

		backward.addPoint(0, 60);
		backward.addPoint(0, 24);

		backwardIndex = purePursuit->addPath(backward);

		// !SECTION

		printf("It isn't: Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
