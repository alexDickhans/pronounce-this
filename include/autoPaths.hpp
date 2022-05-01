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

	int farPlatformToEnterFarHomeZoneIndex = -1;
	int enterFarHomeZoneToMidGoalIndex = -1;
	int farPlatformToRightAllianceDropOffIndex = -1;
	int rightAllianceToFarPlatformIndex = -1;

	// Left Steal
	int leftHomeZoneToLeftNeutralStealIndex = -1;
	int leftNeutralStealToLeftHomeZoneIndex = -1;
	
	// Right steal
	int rightHomeZoneToRightNeutralIndex = -1;
	int rightNeutralToRightHomeZoneIndex = -1;
	int rightHomeZoneToRightNeutralClawIndex = -1;

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
	int rightHomeZoneToLeftHomeZoneIndex = -1;

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

		// SECTION Left Steal

		Path leftHomeZoneToLeftNeutralSteal;
		double leftDistance = 25;
		double leftAngle = toRadians(10);
		// 25 in
		leftHomeZoneToLeftNeutralSteal.addPoint(27, 18);
		leftHomeZoneToLeftNeutralSteal.addPoint(27.0+sin(leftAngle) * leftDistance, 18.0 + cos(leftAngle)*leftDistance);

		leftHomeZoneToLeftNeutralStealIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralSteal);

		Path leftNeutralStealToLeftHomeZone;

		leftNeutralStealToLeftHomeZone.addPoint(27+sin(leftAngle) * leftDistance, 18 + cos(leftAngle)*leftDistance);
		leftNeutralStealToLeftHomeZone.addPoint(25, 18);

		leftNeutralStealToLeftHomeZoneIndex = purePursuit->addPath(leftNeutralStealToLeftHomeZone);

		// !SECTION

		// SECTION Right steal

		QuadraticSplinePath rightHomeZoneToRightNeutral;

		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(107, 19.5), Vector(0.0, 0)));
		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(107, 43), Vector(0.0, 0)));

		rightHomeZoneToRightNeutralIndex = purePursuit->addPath(rightHomeZoneToRightNeutral.getPath(0.1));

		QuadraticSplinePath rightNeutralToRightHomeZone;

		rightNeutralToRightHomeZone.addPoint(SplinePoint(Point(107, 55), Vector(0.0, 0)));
		rightNeutralToRightHomeZone.addPoint(SplinePoint(Point(107, 17), Vector(0.0, 0)));

		rightNeutralToRightHomeZoneIndex = purePursuit->addPath(rightNeutralToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightHomeZoneToRightNeutralClaw;

		rightHomeZoneToRightNeutralClaw.addPoint(SplinePoint(Point(107, 19.5), Vector(0.0, 0)));
		rightHomeZoneToRightNeutralClaw.addPoint(SplinePoint(Point(107, 55), Vector(0.0, 0)));

		rightHomeZoneToRightNeutralClawIndex = purePursuit->addPath(rightHomeZoneToRightNeutralClaw.getPath(0.1));

		// !SECTION

		// SECTION Right right steal

		QuadraticSplinePath farRightHomeZoneToRightNeutral;

		farRightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(121, 18), Vector(0.0, 0)));
		farRightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(113, 47), Vector(0.0, 0)));

		farRightHomeZoneToRightNeutralIndex = purePursuit->addPath(farRightHomeZoneToRightNeutral.getPath(0.1));

		QuadraticSplinePath rightNeutralToFarRightHomeZone;

		rightNeutralToFarRightHomeZone.addPoint(SplinePoint(Point(113, 47), Vector(0.0, 0)));
		rightNeutralToFarRightHomeZone.addPoint(SplinePoint(Point(121, 18), Vector(0.0, 0)));

		rightNeutralToFarRightHomeZoneIndex = purePursuit->addPath(rightNeutralToFarRightHomeZone.getPath(0.1));

		// SECTION Mid Steal

		double midGoalDistance = 32.66;
		double angle = -30;
		
		Path rightHomeZoneToMidNeutral;
		// Distance: 32
		rightHomeZoneToMidNeutral.addPoint(99.5, 18);
		rightHomeZoneToMidNeutral.addPoint(99.5+(sin(toRadians(angle)) * midGoalDistance), 18+(cos(toRadians(angle)) * midGoalDistance));

		rightHomeZoneToMidNeutralIndex = purePursuit->addPath(rightHomeZoneToMidNeutral);

		Path midNeutralToRightHomeZone;

		midNeutralToRightHomeZone.addPoint(99.5+(sin(toRadians(angle)) * midGoalDistance), 18+(cos(toRadians(angle)) * midGoalDistance));
		midNeutralToRightHomeZone.addPoint(105, 35);
		midNeutralToRightHomeZone.addPoint(105, 25);

		midNeutralToRightHomeZoneIndex = purePursuit->addPath(midNeutralToRightHomeZone);

		//!SECTION
			
		// SECTION Left AWP
		
		QuadraticSplinePath leftHomeZoneToLeftAlliance;
		
		leftHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(25, 30), Vector(10.0, M_PI)));
		leftHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(37, 17), Vector(20, M_PI + M_PI_4)));

		leftHomeZoneToLeftAllianceIndex = purePursuit->addPath(leftHomeZoneToLeftAlliance.getPath(0.1));

		QuadraticSplinePath leftAllianceToRightHomeZone;
		
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(35, 15), Vector(10.0, M_PI_4)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(46.8, 42), Vector(20.0, -M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(98, 40.0), Vector(10.0, -M_PI_2)));

		leftAllianceToRightHomeZoneIndex = purePursuit->addPath(leftAllianceToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightHomeZoneToLeftHomeZone;

		rightHomeZoneToLeftHomeZone.addPoint(SplinePoint(Point(110, 32), Vector(0.0, M_PI_2)));
		rightHomeZoneToLeftHomeZone.addPoint(SplinePoint(Point(25, 30), Vector(0.0, M_PI_2)));

		rightHomeZoneToLeftHomeZoneIndex = purePursuit->addPath(rightHomeZoneToLeftHomeZone.getPath(0.2));

		QuadraticSplinePath rightHomeZoneToLeftAlliance;

		rightHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(25, 34), Vector(10, M_PI)));
		rightHomeZoneToLeftAlliance.addPoint(SplinePoint(Point(38, 15), Vector(20, M_PI + (M_PI_2*1.3))));
		
		rightHomeZoneToLeftAllianceIndex = purePursuit->addPath(rightHomeZoneToLeftAlliance.getPath(0.2));

		// !SECTION

		// SECTION Right AWP

		QuadraticSplinePath enterRightHomeZoneToRightAlliance;
		
		enterRightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(98, 46.8), Vector(10.0, -M_PI_2)));
		enterRightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(125, 32), Vector(15, -M_PI_2)));

		enterRightHomeZoneToRightAllianceIndex = purePursuit->addPath(enterRightHomeZoneToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightPlatformToRightAlliance;
		
		rightPlatformToRightAlliance.addPoint(SplinePoint(Point(98, 46.8), Vector(10.0, -M_PI_2)));
		rightPlatformToRightAlliance.addPoint(SplinePoint(Point(125, 35), Vector(15, -M_PI_2)));

		rightPlatformToRightAllianceIndex = purePursuit->addPath(rightPlatformToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightRings;

		rightAllianceToRightRings.addPoint(SplinePoint(Point(127, 37), Vector(10, 0)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(122, 55), Vector(10.0, 0)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(130, 65), Vector(10.0, -M_PI_2)));

		rightAllianceToRightRingsIndex = purePursuit->addPath(rightAllianceToRightRings.getPath(0.1));

		QuadraticSplinePath rightRingsToRightHomeZone;

		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(137, 70), Vector(40, M_PI_2*0.9)));
		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(110, 30), Vector(15, M_PI)));

		rightRingsToRightHomeZoneIndex = purePursuit->addPath(rightRingsToRightHomeZone.getPath(0.1));

		// !SECTION


		//SECTION Skills

		// Left home zone to left neutral goal
		QuadraticSplinePath leftHomeZoneToLeftNeutralGoal("Left Home Zone to Left Neutral Goal");

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(26.5, 11), Vector(15, M_PI_2)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(35, 67), Vector(10, 0)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.1));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone("Left Neutral Goal to Far Home Zone");

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(35, 67), Vector(15, -M_PI_4)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(68, 110), Vector(25, 0.1)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.1));

		QuadraticSplinePath farPlatformToEnterFarHomeZone("Left Home Zone to far right goal drop off");

		farPlatformToEnterFarHomeZone.addPoint(SplinePoint(Point(73, 107), Vector(0.0, 0.0)));
		farPlatformToEnterFarHomeZone.addPoint(SplinePoint(Point(73, 95), Vector(0.0, 0.0)));

		farPlatformToEnterFarHomeZoneIndex = purePursuit->addPath(farPlatformToEnterFarHomeZone.getPath(0.1));

		QuadraticSplinePath enterFarHomeZoneToMidGoal("farLeftAllianceToMidGoal");

		enterFarHomeZoneToMidGoal.addPoint(SplinePoint(Point(73, 95), Vector(5.0, -M_PI)));
		enterFarHomeZoneToMidGoal.addPoint(SplinePoint(Point(70, 70), Vector(10, -M_PI)));

		enterFarHomeZoneToMidGoalIndex = purePursuit->addPath(enterFarHomeZoneToMidGoal.getPath(0.2));

		QuadraticSplinePath midGoalToFarPlatform("midGoalToFarPlatform");

		midGoalToFarPlatform.addPoint(SplinePoint(Point(70, 70), Vector(0.0, 0)));
		midGoalToFarPlatform.addPoint(SplinePoint(Point(80, 110), Vector(0.0, 0)));

		midGoalToFarPlatformIndex = purePursuit->addPath(midGoalToFarPlatform.getPath(0.2));

		QuadraticSplinePath farPlatformToRightAllianceDropOff("farPlatformToFarLeftAllianceDropOff");

		farPlatformToRightAllianceDropOff.addPoint(SplinePoint(Point(80, 107), Vector(0.0, 0)));
		farPlatformToRightAllianceDropOff.addPoint(SplinePoint(Point(70, 80), Vector(0.0, 0)));

		farPlatformToRightAllianceDropOffIndex = purePursuit->addPath(farPlatformToRightAllianceDropOff.getPath(0.1));
		
		QuadraticSplinePath rightAllianceToFarPlatform("farRightDropOffToPlatform");

		rightAllianceToFarPlatform.addPoint(SplinePoint(Point(70, 80), Vector(7.0, 0)));
		rightAllianceToFarPlatform.addPoint(SplinePoint(Point(70, 107), Vector(7.0, 0)));

		rightAllianceToFarPlatformIndex = purePursuit->addPath(rightAllianceToFarPlatform.getPath(0.1));

		QuadraticSplinePath platformToEnterFarRightHomeZone("platformToEnterFarRightHomeZone");

		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(70, 115), Vector(7.0, M_PI)));
		platformToEnterFarRightHomeZone.addPoint(SplinePoint(Point(70, 100), Vector(7.0, M_PI)));

		platformToEnterFarRightHomeZoneIndex = purePursuit->addPath(platformToEnterFarRightHomeZone.getPath(0.1));

		QuadraticSplinePath enterRightHomeZoneToFarRightAlliance("enterRightHomeZoneToFarRightAlliance");

		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(70, 100), Vector(20.0, -M_PI_2)));
		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(85, 100), Vector(20.0, -M_PI_2)));
		enterRightHomeZoneToFarRightAlliance.addPoint(SplinePoint(Point(114, 122), Vector(35, M_PI_2*1.1)));

		enterRightHomeZoneToFarRightAllianceIndex = purePursuit->addPath(enterRightHomeZoneToFarRightAlliance.getPath(0.1));

		QuadraticSplinePath farRightAllianceToRightHomeZone("farRightAllianceToRightHomeZone");

		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(105.7, 130), Vector(20.0, M_PI_4 * 1.3 + M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(135, 70), Vector(20.0, M_PI)));
		farRightAllianceToRightHomeZone.addPoint(SplinePoint(Point(125, 20), Vector(20.0, M_PI)));

		farRightAllianceToRightHomeZoneIndex = purePursuit->addPath(farRightAllianceToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightHomeZoneToRightAlliance("rightHomeZoneToRightAlliance");

		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(105.7, 15), Vector(5.0, -M_PI_4)));
		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(145, 35), Vector(10.0, -M_PI_4 * 0.6)));

		rightHomeZoneToRightAllianceIndex = purePursuit->addPath(rightHomeZoneToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightNeutral("rightAllianceToRightNeutral");

		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(145, 35), Vector(10.0, 0)));
		rightAllianceToRightNeutral.addPoint(SplinePoint(Point(122, 62), Vector(5.0, M_PI_4)));

		rightAllianceToRightNeutralIndex = purePursuit->addPath(rightAllianceToRightNeutral.getPath(0.1));

		QuadraticSplinePath rightNeutralToPlatform("rightNeutralToPlatform");

		rightNeutralToPlatform.addPoint(SplinePoint(Point(122, 62), Vector(15.0, 0)));
		rightNeutralToPlatform.addPoint(SplinePoint(Point(100, 115), Vector(15.0, 0)));

		rightNeutralToPlatformIndex = purePursuit->addPath(rightNeutralToPlatform.getPath(0.1));

		QuadraticSplinePath rightPlatformToFarRightGoalDropOff2("rightPlatformToFarRightGoalDropOff2");

		rightPlatformToFarRightGoalDropOff2.addPoint(SplinePoint(Point(100, 115), Vector(15.0, M_PI)));
		rightPlatformToFarRightGoalDropOff2.addPoint(SplinePoint(Point(110, 100), Vector(15.0, M_PI)));

		rightPlatformToFarRightGoalDropOff2Index = purePursuit->addPath(rightPlatformToFarRightGoalDropOff2.getPath(0.1));

		QuadraticSplinePath farRightGoalDropOff2ToFarLeftAllianceDropOff("farRightGoalDropOff2ToFarLeftAllianceDropOff");

		farRightGoalDropOff2ToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(110, 105.7), Vector(15.0, M_PI_2)));
		farRightGoalDropOff2ToFarLeftAllianceDropOff.addPoint(SplinePoint(Point(25, 105.7), Vector(15.0, M_PI_2)));

		farRightGoalDropOff2ToFarLeftAllianceDropOffIndex = purePursuit->addPath(farRightGoalDropOff2ToFarLeftAllianceDropOff.getPath(0.1));

		QuadraticSplinePath farLeftAllianceDropOffToFarRightGoalDropOff2("farLeftAllianceDropOffToFarRightGoalDropOff2");

		farLeftAllianceDropOffToFarRightGoalDropOff2.addPoint(SplinePoint(Point(25, 105.7), Vector(15.0, -M_PI_2)));
		farLeftAllianceDropOffToFarRightGoalDropOff2.addPoint(SplinePoint(Point(110, 105.7), Vector(15.0, -M_PI_2)));

		farLeftAllianceDropOffToFarRightGoalDropOff2Index = purePursuit->addPath(farLeftAllianceDropOffToFarRightGoalDropOff2.getPath(0.1));

		QuadraticSplinePath farLeftAllianceDropOffToPlatform("farLeftAllianceDropOffToPlatform");

		farLeftAllianceDropOffToPlatform.addPoint(SplinePoint(Point(80, 105.7), Vector(15.0, 0)));
		farLeftAllianceDropOffToPlatform.addPoint(SplinePoint(Point(80, 115), Vector(15.0, 0)));

		farLeftAllianceDropOffToPlatformIndex = purePursuit->addPath(farLeftAllianceDropOffToPlatform.getPath(0.1));

		QuadraticSplinePath platformToEnterFarLeftHomeZone("platformToEnterFarLeftHomeZone");

		platformToEnterFarLeftHomeZone.addPoint(SplinePoint(Point(80, 115), Vector(15.0, M_PI)));
		platformToEnterFarLeftHomeZone.addPoint(SplinePoint(Point(80, 105.7), Vector(15.0, M_PI)));

		platformToEnterFarLeftHomeZoneIndex = purePursuit->addPath(platformToEnterFarLeftHomeZone.getPath(0.1));

		QuadraticSplinePath enterFarLeftHomeZoneToNearRightPlatform("enterFarLeftHomeZoneToNearRightPlatform");

		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(100, 105.7), Vector(25, -M_PI_2)));
		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(135, 45), Vector(10, M_PI*1.2)));
		enterFarLeftHomeZoneToNearRightPlatform.addPoint(SplinePoint(Point(132, 14), Vector(25, M_PI_2)));

		enterFarLeftHomeZoneToNearRightPlatformIndex = purePursuit->addPath(enterFarLeftHomeZoneToNearRightPlatform.getPath(0.1));

		// !SECTION


		printf("Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
