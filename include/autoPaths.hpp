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
	int farRightDropOffToFarLeftAllianceGoalIndex = -1;
	int farLeftAllianceToMidGoalIndex = -1;
	int midGoalToFarPlatformIndex = -1;
	int farPlatformToFarLeftAllianceDropOffIndex = -1;
	int farLeftAllianceDropOffToFarRightDropOffIndex = -1;
	int farRightDropOffToPlatformIndex = -1;
	int platformToEnterFarRightHomeZoneIndex = -1;
	int enterRightHomeZoneToFarRightAllianceIndex = -1;
	int farRightAllianceToRightHomeZoneIndex = -1;
	int rightHomeZoneToRightAllianceIndex = -1;
	int rightAllianceToRightNeutralIndex = -1;
	int rightNeutralToPlatformIndex = -1;
	int rightPlatformToFarRightGoalDropOff2Index = -1;
	int farRightGoalDropOff2ToFarLeftAllianceDropOffIndex = -1;
	int farLeftAllianceDropOffToFarRightGoalDropOff2Index = -1;
	int farLeftAllianceDropOffToPlatformIndex = -1;
	int platformToEnterFarLeftHomeZoneIndex = -1;
	int enterFarLeftHomeZoneToNearRightPlatformIndex = -1;

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

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(26.5, 11), Vector(30, M_PI_2)));
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

		QuadraticSplinePath farRightDropOffToFarLeftAllianceGoal;

		farRightDropOffToFarLeftAllianceGoal.addPoint(SplinePoint(Point(82, 102), Vector(20.0, M_PI_2)));
		farRightDropOffToFarLeftAllianceGoal.addPoint(SplinePoint(Point(20, 105.7), Vector(20.0, M_PI_2)));

		farRightDropOffToFarLeftAllianceGoalIndex = purePursuit->addPath(farRightDropOffToFarLeftAllianceGoal.getPath(0.05));

		QuadraticSplinePath farLeftAllianceToMidGoal("farRightDropOffToFarLeftAllianceGoal");

		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(15, 105.7), Vector(5, -M_PI_2)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(23.2, 85), Vector(15, M_PI)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(35, 70), Vector(10.0, -M_PI_2)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(65, 70), Vector(5.0, -M_PI_2)));

		farLeftAllianceToMidGoalIndex = purePursuit->addPath(farLeftAllianceToMidGoal.getPath(0.05));

		QuadraticSplinePath midGoalToFarPlatform("midGoalToFarPlatform");

		midGoalToFarPlatform.addPoint(SplinePoint(Point(60, 70), Vector(20, 0)));
		midGoalToFarPlatform.addPoint(SplinePoint(Point(70, 115), Vector(40, 0)));

		midGoalToFarPlatformIndex = purePursuit->addPath(midGoalToFarPlatform.getPath(0.05));

		QuadraticSplinePath farPlatformToFarLeftAllianceDropOff("farPlatformToFarLeftAllianceDropOff");

		farPlatformToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(70, 115), Vector(5, M_PI)));
		farPlatformToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(58.9, 105.7), Vector(7.0, M_PI_2)));

		farPlatformToFarLeftAllianceDropOffIndex = purePursuit->addPath(farPlatformToFarLeftAllianceDropOff.getPath(0.05));

		QuadraticSplinePath farLeftAllianceDropOffToFarRightDropOff("farPlatformToFarLeftAllianceDropOff");

		farLeftAllianceDropOffToFarRightDropOff.addPoint(SplinePoint(Point(58.9, 105.7), Vector(7.0, -M_PI_2)));
		farLeftAllianceDropOffToFarRightDropOff.addPoint(SplinePoint(Point(85, 105.7), Vector(7.0, -M_PI_2)));

		farLeftAllianceDropOffToFarRightDropOffIndex = purePursuit->addPath(farLeftAllianceDropOffToFarRightDropOff.getPath(0.05));
		
		QuadraticSplinePath farRightDropOffToPlatform("farRightDropOffToPlatform");

		farRightDropOffToPlatform.addPoint(SplinePoint(Point(85, 105.7), Vector(7.0, 0)));
		farRightDropOffToPlatform.addPoint(SplinePoint(Point(85, 115), Vector(7.0, 0)));

		farRightDropOffToPlatformIndex = purePursuit->addPath(farRightDropOffToPlatform.getPath(0.05));

		QuadraticSplinePath platformToEnterFarRightHomeZone("platformToEnterFarRightHomeZone");

		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(85, 115), Vector(7.0, M_PI)));
		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(85, 105.7), Vector(7.0, M_PI)));

		platformToEnterFarRightHomeZoneIndex = purePursuit->addPath(platformToEnterFarRightHomeZone.getPath(0.05));

		QuadraticSplinePath enterRightHomeZoneToFarRightAlliance("enterRightHomeZoneToFarRightAlliance");

		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(85, 105.7), Vector(20.0, -M_PI_2)));
		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(105.7, 130), Vector(20.0, M_PI_4 * 1.3)));

		enterRightHomeZoneToFarRightAllianceIndex = purePursuit->addPath(enterRightHomeZoneToFarRightAlliance.getPath(0.05));

		QuadraticSplinePath farRightAllianceToRightHomeZone("farRightAllianceToRightHomeZone");

		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(105.7, 130), Vector(20.0, M_PI_4 * 1.3 + M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(122, 70), Vector(20.0, M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(105.7, 30), Vector(20.0, M_PI)));

		farRightAllianceToRightHomeZoneIndex = purePursuit->addPath(farRightAllianceToRightHomeZone.getPath(0.05));

		QuadraticSplinePath rightHomeZoneToRightAlliance("rightHomeZoneToRightAlliance");

		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(105.7, 15), Vector(5.0, -M_PI_4)));
		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(125, 30), Vector(10.0, -M_PI_4 * 0.6)));

		rightHomeZoneToRightAllianceIndex = purePursuit->addPath(rightHomeZoneToRightAlliance.getPath(0.05));

		QuadraticSplinePath rightAllianceToRightNeutral("rightAllianceToRightNeutral");

		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(125, 30), Vector(10.0, 0)));
		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(110, 62), Vector(5.0, M_PI_4)));

		rightAllianceToRightNeutralIndex = purePursuit->addPath(rightAllianceToRightNeutral.getPath(0.05));

		QuadraticSplinePath rightNeutralToPlatform("rightNeutralToPlatform");

		rightNeutralToPlatform.addPoint(SplinePoint(Point(110, 62), Vector(15.0, 0)));
		rightNeutralToPlatform.addPoint(SplinePoint(Point(80, 115), Vector(15.0, 0)));

		rightNeutralToPlatformIndex = purePursuit->addPath(rightNeutralToPlatform.getPath(0.05));

		QuadraticSplinePath rightPlatformToFarRightGoalDropOff2("rightPlatformToFarRightGoalDropOff2");

		rightPlatformToFarRightGoalDropOff2.addPoint(SplinePoint(Point(80, 115), Vector(15.0, M_PI)));
		rightPlatformToFarRightGoalDropOff2.addPoint(SplinePoint(Point(80, 105.7), Vector(15.0, M_PI)));

		rightPlatformToFarRightGoalDropOff2Index = purePursuit->addPath(rightPlatformToFarRightGoalDropOff2.getPath(0.05));

		QuadraticSplinePath farRightGoalDropOff2ToFarLeftAllianceDropOff("farRightGoalDropOff2ToFarLeftAllianceDropOff");

		farRightGoalDropOff2ToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(80, 105.7), Vector(15.0, M_PI_2)));
		farRightGoalDropOff2ToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(60, 105.7), Vector(15.0, M_PI_2)));

		farRightGoalDropOff2ToFarLeftAllianceDropOffIndex = purePursuit->addPath(farRightGoalDropOff2ToFarLeftAllianceDropOff.getPath(0.05));

		QuadraticSplinePath farLeftAllianceDropOffToFarRightGoalDropOff2("farLeftAllianceDropOffToFarRightGoalDropOff2");

		farLeftAllianceDropOffToFarRightGoalDropOff2.addPoint(SplinePoint(Point(60, 105.7), Vector(15.0, -M_PI_2)));
		farLeftAllianceDropOffToFarRightGoalDropOff2.addPoint(SplinePoint(Point(80, 105.7), Vector(15.0, -M_PI_2)));

		farLeftAllianceDropOffToFarRightGoalDropOff2Index = purePursuit->addPath(farLeftAllianceDropOffToFarRightGoalDropOff2.getPath(0.05));

		QuadraticSplinePath farLeftAllianceDropOffToPlatform("farLeftAllianceDropOffToPlatform");

		farLeftAllianceDropOffToPlatform.addPoint(SplinePoint(Point(60, 105.7), Vector(15.0, 0)));
		farLeftAllianceDropOffToPlatform.addPoint(SplinePoint(Point(60, 115), Vector(15.0, 0)));

		farLeftAllianceDropOffToPlatformIndex = purePursuit->addPath(farLeftAllianceDropOffToPlatform.getPath(0.05));

		QuadraticSplinePath platformToEnterFarLeftHomeZone("platformToEnterFarLeftHomeZone");

		platformToEnterFarLeftHomeZone.addPoint(SplinePoint(Point(60, 115), Vector(15.0, M_PI)));
		platformToEnterFarLeftHomeZone.addPoint(SplinePoint(Point(60, 105.7), Vector(15.0, M_PI)));

		platformToEnterFarLeftHomeZoneIndex = purePursuit->addPath(platformToEnterFarLeftHomeZone.getPath(0.05));

		QuadraticSplinePath enterFarLeftHomeZoneToNearRightPlatform("enterFarLeftHomeZoneToNearRightPlatform");

		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(60, 105.7), Vector(25, -M_PI_2)));
		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(85, 70), Vector(25, -M_PI_2)));
		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(105.7, 45), Vector(10, M_PI*1.2)));
		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(100, 14), Vector(25, M_PI_2)));

		enterFarLeftHomeZoneToNearRightPlatformIndex = purePursuit->addPath(enterFarLeftHomeZoneToNearRightPlatform.getPath(0.05));

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
