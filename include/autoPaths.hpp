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
	int zeroPathIndex = 0;
	int testPathIndex;

	// Skills
	int leftHomeZoneToLeftNeutralGoalIndex = -1;
	int leftNeutralGoalToFarHomeZoneIndex = -1;
	int farLeftHomeZoneToFarRightGoalDropOffIndex = -1;
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

	// Left Steal
	int leftHomeZoneToLeftNeutralStealIndex = -1;
	int leftNeutralStealToLeftHomeZoneIndex = -1;
	
	// Right steal
	int rightHomeZoneToRightNeutralIndex = -1;
	int rightNeutralToRightHomeZoneIndex = -1;

	// Right(Right position) steal
	int farRightHomeZoneToRightNeutralIndex = -1;
	int rightNeutralToFarRightHomeZoneIndex = -1;

	// Right mid steal
	int rightHomeZoneToMidNeutralIndex = -1;
	int midNeutralToRightHomeZoneIndex = -1;

	// Left AWP
	int leftHomeZoneToLeftAllianceIndex = -1;
	int leftAllianceToRightHomeZoneIndex = -1;
	int rightHomeZoneToLeftAllianceIndex = -1;
	int leftAllianceToPreloadsIndex = -1;
	int preloadsToLeftAllianceIndex = -1;

	// Right AWP
	int enterRightHomeZoneToRightAllianceIndex = -1;
	int rightPlatformToRightAllianceIndex = -1;
	int rightAllianceToRightRingsIndex = -1;
	int rightAllianceToLeftAllianceIndex = -1;
	int rightRingsToRightHomeZoneIndex = -1;

	int wiggleIndex = -1;

	void autoPaths(PurePursuit* purePursuit) {
		printf("Why the unknown error, is it in this function??\n");

		// Default pure pursuit profile
		PurePursuitProfile defaultProfile(new PID(20, 0.0, 2.0), new PID(60.0, 0.0, 5.0), 10.0);
		purePursuit->getPurePursuitProfileManager().setDefaultProfile(defaultProfile);
		
		Path zeroPath = Path();

		zeroPath.addPoint(0, 0);
		zeroPath.addPoint(0, 0);

		zeroPathIndex = purePursuit->addPath(zeroPath);

		//SECTION: Test path

		Path testPath = Path();

		testPath.addPoint(0, 0);
		testPath.addPoint(0, 100);

		testPathIndex = purePursuit->addPath(testPath);

		// !SECTION

		//SECTION Skills

		// Left home zone to left neutral goal
		QuadraticSplinePath leftHomeZoneToLeftNeutralGoal("Left Home Zone to Left Neutral Goal");

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(26.5, 11), Vector(20, M_PI_2)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(38, 67), Vector(10, 0)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.05));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone("Left Neutral Goal to Far Home Zone");

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(35, 72), Vector(25, -M_PI_4)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(78, 110), Vector(25, 0.1)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.1));

		QuadraticSplinePath farLeftHomeZoneToFarRightGoalDropOff("Left Home Zone to far right goal drop off");

		farLeftHomeZoneToFarRightGoalDropOff.addPoint(SplinePoint(Point(78, 107), Vector(0.0, 0.0)));
		farLeftHomeZoneToFarRightGoalDropOff.addPoint(SplinePoint(Point(83, 107), Vector(0.0, 0.0)));

		farLeftHomeZoneToFarRightGoalDropOffIndex = purePursuit->addPath(farLeftHomeZoneToFarRightGoalDropOff.getPath(0.05));

		QuadraticSplinePath farRightDropOffToFarLeftAllianceGoal;

		farRightDropOffToFarLeftAllianceGoal.addPoint(SplinePoint(Point(82, 102), Vector(20.0, M_PI_2)));
		farRightDropOffToFarLeftAllianceGoal.addPoint(SplinePoint(Point(20, 105), Vector(20.0, M_PI_2)));

		farRightDropOffToFarLeftAllianceGoalIndex = purePursuit->addPath(farRightDropOffToFarLeftAllianceGoal.getPath(0.05));

		QuadraticSplinePath farLeftAllianceToMidGoal("farLeftAllianceToMidGoal");

		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(20, 105), Vector(5, -M_PI_2)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(33, 85), Vector(15, M_PI)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(40, 75), Vector(10.0, -M_PI_2)));
		farLeftAllianceToMidGoal.addPoint(SplinePoint(Point(70, 67), Vector(5.0, -M_PI_2)));

		farLeftAllianceToMidGoalIndex = purePursuit->addPath(farLeftAllianceToMidGoal.getPath(0.05));

		QuadraticSplinePath midGoalToFarPlatform("midGoalToFarPlatform");

		midGoalToFarPlatform.addPoint(SplinePoint(Point(70, 70), Vector(0.0, 0)));
		midGoalToFarPlatform.addPoint(SplinePoint(Point(80, 108), Vector(0.0, 0)));

		midGoalToFarPlatformIndex = purePursuit->addPath(midGoalToFarPlatform.getPath(0.05));

		QuadraticSplinePath farPlatformToFarLeftAllianceDropOff("farPlatformToFarLeftAllianceDropOff");

		farPlatformToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(85, 105.7), Vector(0.0, 0)));
		farPlatformToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(80, 102), Vector(0.0, 0)));

		farPlatformToFarLeftAllianceDropOffIndex = purePursuit->addPath(farPlatformToFarLeftAllianceDropOff.getPath(0.05));

		QuadraticSplinePath farLeftAllianceDropOffToFarRightDropOff("farLeftAllianceDropOffToFarRightDropOff");

		farLeftAllianceDropOffToFarRightDropOff.addPoint(SplinePoint(Point(58.9, 105.7), Vector(7.0, -M_PI_2)));
		farLeftAllianceDropOffToFarRightDropOff.addPoint(SplinePoint(Point(88, 105), Vector(7.0, -M_PI_2)));

		farLeftAllianceDropOffToFarRightDropOffIndex = purePursuit->addPath(farLeftAllianceDropOffToFarRightDropOff.getPath(0.05));
		
		QuadraticSplinePath farRightDropOffToPlatform("farRightDropOffToPlatform");

		farRightDropOffToPlatform.addPoint(SplinePoint(Point(88, 105.7), Vector(7.0, 0)));
		farRightDropOffToPlatform.addPoint(SplinePoint(Point(88, 110), Vector(7.0, 0)));

		farRightDropOffToPlatformIndex = purePursuit->addPath(farRightDropOffToPlatform.getPath(0.05));

		QuadraticSplinePath platformToEnterFarRightHomeZone("platformToEnterFarRightHomeZone");

		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(85, 115), Vector(7.0, M_PI)));
		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(85, 105.7), Vector(7.0, M_PI)));

		platformToEnterFarRightHomeZoneIndex = purePursuit->addPath(platformToEnterFarRightHomeZone.getPath(0.05));

		QuadraticSplinePath enterRightHomeZoneToFarRightAlliance("enterRightHomeZoneToFarRightAlliance");

		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(85, 105.7), Vector(20.0, -M_PI_2)));
		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(107, 125), Vector(40.0, M_PI_4 * 1.5)));

		enterRightHomeZoneToFarRightAllianceIndex = purePursuit->addPath(enterRightHomeZoneToFarRightAlliance.getPath(0.05));

		QuadraticSplinePath farRightAllianceToRightHomeZone("farRightAllianceToRightHomeZone");

		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(105.7, 130), Vector(20.0, M_PI_4 * 1.3 + M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(135, 70), Vector(20.0, M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(125, 20), Vector(20.0, M_PI)));

		farRightAllianceToRightHomeZoneIndex = purePursuit->addPath(farRightAllianceToRightHomeZone.getPath(0.05));

		QuadraticSplinePath rightHomeZoneToRightAlliance("rightHomeZoneToRightAlliance");

		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(105.7, 15), Vector(5.0, -M_PI_4)));
		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(145, 35), Vector(10.0, -M_PI_4 * 0.6)));

		rightHomeZoneToRightAllianceIndex = purePursuit->addPath(rightHomeZoneToRightAlliance.getPath(0.05));

		QuadraticSplinePath rightAllianceToRightNeutral("rightAllianceToRightNeutral");

		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(145, 35), Vector(10.0, 0)));
		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(134, 62), Vector(5.0, M_PI_4)));

		rightAllianceToRightNeutralIndex = purePursuit->addPath(rightAllianceToRightNeutral.getPath(0.05));

		QuadraticSplinePath rightNeutralToPlatform("rightNeutralToPlatform");

		rightNeutralToPlatform.addPoint(SplinePoint(Point(134, 62), Vector(15.0, 0)));
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

		// SECTION Left Steal

		Path leftHomeZoneToLeftNeutralSteal;

		leftHomeZoneToLeftNeutralSteal.addPoint(27, 18);
		leftHomeZoneToLeftNeutralSteal.addPoint(33, 48);

		leftHomeZoneToLeftNeutralStealIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralSteal);

		Path leftNeutralStealToLeftHomeZone;

		leftNeutralStealToLeftHomeZone.addPoint(33, 48);
		leftNeutralStealToLeftHomeZone.addPoint(25, 18);

		leftNeutralStealToLeftHomeZoneIndex = purePursuit->addPath(leftNeutralStealToLeftHomeZone);

		// !SECTION

		// SECTION Right steal

		QuadraticSplinePath rightHomeZoneToRightNeutral;

		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(107, 19.5), Vector(0.0, 0)));
		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(107, 46), Vector(0.0, 0)));

		rightHomeZoneToRightNeutralIndex = purePursuit->addPath(rightHomeZoneToRightNeutral.getPath(0.01));

		QuadraticSplinePath rightNeutralToRightHomeZone;

		rightNeutralToRightHomeZone.addPoint(SplinePoint(Point(107, 46), Vector(0.0, 0)));
		rightNeutralToRightHomeZone.addPoint(SplinePoint(Point(107, 17), Vector(0.0, 0)));

		rightNeutralToRightHomeZoneIndex = purePursuit->addPath(rightNeutralToRightHomeZone.getPath(0.01));

		// !SECTION

		// SECTION Right right steal

		QuadraticSplinePath farRightHomeZoneToRightNeutral;

		farRightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(121, 18), Vector(0.0, 0)));
		farRightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(113, 47), Vector(0.0, 0)));

		farRightHomeZoneToRightNeutralIndex = purePursuit->addPath(farRightHomeZoneToRightNeutral.getPath(0.01));

		QuadraticSplinePath rightNeutralToFarRightHomeZone;

		rightNeutralToFarRightHomeZone.addPoint(SplinePoint(Point(113, 47), Vector(0.0, 0)));
		rightNeutralToFarRightHomeZone.addPoint(SplinePoint(Point(121, 18), Vector(0.0, 0)));

		rightNeutralToFarRightHomeZoneIndex = purePursuit->addPath(rightNeutralToFarRightHomeZone.getPath(0.01));

		// SECTION Mid Steal

		Path rightHomeZoneToMidNeutral;
		// Distance: 32
		rightHomeZoneToMidNeutral.addPoint(99.5, 18);
		rightHomeZoneToMidNeutral.addPoint(82, 47);

		rightHomeZoneToMidNeutralIndex = purePursuit->addPath(rightHomeZoneToMidNeutral);

		Path midNeutralToRightHomeZone;

		midNeutralToRightHomeZone.addPoint(82, 47);
		midNeutralToRightHomeZone.addPoint(110, 25);

		midNeutralToRightHomeZoneIndex = purePursuit->addPath(midNeutralToRightHomeZone);

		//!SECTION
			
		// SECTION Left AWP
		
		QuadraticSplinePath leftHomeZoneToLeftAlliance;
		
		leftHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(25, 30), Vector(10.0, M_PI)));
		leftHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(35, 20), Vector(15.0, M_PI + M_PI_4)));

		leftHomeZoneToLeftAllianceIndex = purePursuit->addPath(leftHomeZoneToLeftAlliance.getPath(0.01));

		QuadraticSplinePath leftAllianceToRightHomeZone;
		
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(35, 15), Vector(10.0, M_PI_4)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(46.8, 46.8), Vector(20.0, -M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(98, 40.0), Vector(10.0, -M_PI_2)));

		leftAllianceToRightHomeZoneIndex = purePursuit->addPath(leftAllianceToRightHomeZone.getPath(0.01));

		QuadraticSplinePath rightHomeZoneToLeftAlliance;

		rightHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(98, 35), Vector(20.0, M_PI_2)));
		rightHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(46, 35), Vector(10.0, M_PI_2)));
		rightHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(30, 17), Vector(10.0, M_PI + M_PI_4)));
		
		rightHomeZoneToLeftAllianceIndex = purePursuit->addPath(rightHomeZoneToLeftAlliance.getPath(0.01));

		QuadraticSplinePath leftAllianceToPreloads;

		leftAllianceToPreloads.addPoint(SplinePoint(Point(35, 15), Vector(10.0, M_PI_2)));
		leftAllianceToPreloads.addPoint(SplinePoint(Point(15, 15), Vector(10.0, M_PI_2)));

		leftAllianceToPreloadsIndex = purePursuit->addPath(leftAllianceToPreloads.getPath(0.01));

		QuadraticSplinePath preloadsToLeftAlliance;

		preloadsToLeftAlliance.addPoint(SplinePoint(Point(35, 15), Vector(10.0, M_PI_2)));
		preloadsToLeftAlliance.addPoint(SplinePoint(Point(15, 15), Vector(10.0, M_PI_2)));

		preloadsToLeftAllianceIndex = purePursuit->addPath(preloadsToLeftAlliance.getPath(0.01));

		// !SECTION

		// SECTION Right AWP

		QuadraticSplinePath enterRightHomeZoneToRightAlliance;
		
		enterRightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(98, 46.8), Vector(10.0, -M_PI_2)));
		enterRightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(123, 37), Vector(15, - M_PI_2 - 0.4)));

		enterRightHomeZoneToRightAllianceIndex = purePursuit->addPath(enterRightHomeZoneToRightAlliance.getPath(0.01));

		QuadraticSplinePath rightPlatformToRightAlliance;
		
		rightPlatformToRightAlliance.addPoint(SplinePoint(Point(98, 40), Vector(10.0, -M_PI_2)));
		rightPlatformToRightAlliance.addPoint(SplinePoint(Point(122, 35), Vector(15, - M_PI_2)));

		rightPlatformToRightAllianceIndex = purePursuit->addPath(rightPlatformToRightAlliance.getPath(0.01));

		QuadraticSplinePath rightAllianceToRightRings;

		rightAllianceToRightRings.addPoint(SplinePoint(Point(120, 37), Vector(5, M_PI_4)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(117.5, 55), Vector(10.0, 0)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(117.5, 60), Vector(10.0, 0)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(120, 70), Vector(10.0, -M_PI_2)));

		rightAllianceToRightRingsIndex = purePursuit->addPath(rightAllianceToRightRings.getPath(0.01));

		QuadraticSplinePath rightRingsToRightHomeZone;

		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(137, 70), Vector(20.0, M_PI_2)));
		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(120, 25), Vector(15, M_PI)));

		rightRingsToRightHomeZoneIndex = purePursuit->addPath(rightRingsToRightHomeZone.getPath(0.01));

		QuadraticSplinePath rightAllianceToLeftAlliance;
		
		rightAllianceToLeftAlliance.addPoint(SplinePoint(Point(122, 30), Vector(10.0, M_PI_4*0.7)));
		rightAllianceToLeftAlliance.addPoint(SplinePoint(Point(80, 35), Vector(10.0, M_PI_2)));
		rightAllianceToLeftAlliance.addPoint(SplinePoint(Point(38, 35), Vector(30, M_PI_2)));
		rightAllianceToLeftAlliance.addPoint(SplinePoint(Point(32, 15), Vector(30, -M_PI_2)));

		rightAllianceToLeftAllianceIndex = purePursuit->addPath(rightAllianceToLeftAlliance.getPath(0.01));

		// !SECTION

		QuadraticSplinePath wiggle;

		wiggle.addPoint(SplinePoint(Point(20, 0), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(10, 20), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(20, 40), Vector(10.0, 0.0)));
		wiggle.addPoint(SplinePoint(Point(10, 60), Vector(10.0, 0.0)));

		wiggleIndex = purePursuit->addPath(wiggle.getPath(0.01));

		printf("Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
