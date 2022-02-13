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
	int leftAllianceToRightHomeZoneIndex;
	int rightHomeZoneToRightAllianceIndex;
	int rightAllianceToRightHomeZoneIndex;
	int rightAllianceToRightRingsIndex;

	// Skills
	int leftHomeZoneToLeftNeutralGoalIndex;
	int leftNeutralGoalToFarHomeZoneIndex;
	int farHomeZoneToMidNeutralGoalIndex;
	int farHomeZoneToEnterFarHomeZoneIndex;
	int midNeutralGoalToPlatformIndex;
	int farPlatformToDropOffGoalIndex;
	int goalDropOffToFarLeftAllianceIndex;
	int farLeftAllianceToLeftRingsIndex;
	int leftRingsToGoalDropIndex;
	int rightHomeZoneToFarRightAllianceGoalIndex;
	int farRightAllianceGoalToNearPreloadsIndex;
	int nearPlatformAlignIndex;
	int leftHomeZoneToPlatformIndex;
	int enterHomeZoneToRightHomeZoneIndex;

	// Right steal right
	int rightHomeZoneToRightNeutralIndex = -1;
	int rightNeutralToRightAllianceGoalIndex = -1;
	int rightAllianceGoalToRightRingsIndex = -1;
	int rightRingsToRightHomeZoneIndex = -1;

	int forwardIndex = -1;
	int backwardIndex = -1;

	void autoPaths(PurePursuit* purePursuit) {
		// Default pure pursuit profile
		PurePursuitProfile defaultProfile(new PID(20, 0.0, 2.0), new PID(60.0, 0.0, 5.0), 10.0);
		purePursuit->getPurePursuitProfileManager().setDefaultProfile(defaultProfile);

		//SECTION: Test path
		Path testPath = Path();

		testPath.addPoint(0, 0);
		testPath.addPoint(0, 100);

		testPathIndex = purePursuit->addPath(testPath);

		// !SECTION

		// SECTION Left AWP
		QuadraticSplinePath leftAllianceToRightHomeZone = QuadraticSplinePath();

		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(30.0, 11.4), Vector(0, M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(15.2, 35.0), Vector(0, 0.0)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(46.4, 46.4), Vector(0, M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(80, 46.4), Vector(0, M_PI_2)));

		leftAllianceToRightHomeZoneIndex = purePursuit->addPath(leftAllianceToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightHomeZoneToRightAlliance = QuadraticSplinePath();

		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(80, 46.4), Vector(15.0, -M_PI_2 - M_PI_4)));
		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(129.0, 30), Vector(15.0, -M_PI_2 - M_PI_4)));

		rightHomeZoneToRightAllianceIndex = purePursuit->addPath(rightHomeZoneToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightHomeZone = QuadraticSplinePath();

		rightAllianceToRightHomeZone.addPoint(SplinePoint(Point(130.0, 30), Vector(15.0, -M_PI_2 - M_PI_4)));
		rightAllianceToRightHomeZone.addPoint(SplinePoint(Point(100, 30), Vector(15.0, -M_PI_2 - M_PI_4)));

		rightAllianceToRightHomeZoneIndex = purePursuit->addPath(rightAllianceToRightHomeZone.getPath(0.1));

		// !SECTION

		//SECTION Skills

		// Left home zone to left neutral goal
		QuadraticSplinePath leftHomeZoneToLeftNeutralGoal("Left Home Zone to Left Neutral Goal");

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(23.2, 11.4), Vector(25, M_PI_4 * 1.5)));
		// leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(48, 46.8), Vector(15, 0)));
		// leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(30, 70), Vector(20, M_PI_4*0.75)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(33, 61), Vector(20, 0)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.01));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone("Left Neutral Goal to Far Home Zone");

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(33, 68), Vector(10, 0)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(68, 110), Vector(30, 0)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.01));

		Path farHomeZoneToEnterFarHomeZone("Far Home Zone to Enter Far Home Zone");

		farHomeZoneToEnterFarHomeZone.addPoint(70.3, 110);
		farHomeZoneToEnterFarHomeZone.addPoint(70.3, 100);

		farHomeZoneToEnterFarHomeZoneIndex = purePursuit->addPath(farHomeZoneToEnterFarHomeZone);

		Path farHomeZoneToMidNeutralGoal("Far Home Zone to Mid Neutral Goal");

		farHomeZoneToMidNeutralGoal.addPoint(64, 110);
		farHomeZoneToMidNeutralGoal.addPoint(64, 80);

		farHomeZoneToMidNeutralGoalIndex = purePursuit->addPath(farHomeZoneToMidNeutralGoal);

		QuadraticSplinePath midNeutralGoalToPlatform("Mid Neutral Goal to Platform");

		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(70, 75), Vector(30, 0)));
		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(75, 110), Vector(30, 0)));

		midNeutralGoalToPlatformIndex = purePursuit->addPath(midNeutralGoalToPlatform.getPath(0.1));

		QuadraticSplinePath farPlatformToDropOffGoal("Far Platform to Drop Off Goal");

		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(75, 107), Vector(5, 0)));
		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(75, 80), Vector(5, 0)));

		farPlatformToDropOffGoalIndex = purePursuit->addPath(farPlatformToDropOffGoal.getPath(0.01));

		QuadraticSplinePath goalDropOffToFarLeftAlliance("Goal Drop Off to Far Left Alliance");

		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(65, 80), Vector(15, 0)));
		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4)));

		goalDropOffToFarLeftAllianceIndex = purePursuit->addPath(goalDropOffToFarLeftAlliance.getPath(0.01));

		QuadraticSplinePath farLeftAllianceToLeftRings("Far Left Alliance to Left Rings");

		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4 + M_PI)));
		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, M_PI)));

		farLeftAllianceToLeftRingsIndex = purePursuit->addPath(farLeftAllianceToLeftRings.getPath(0.01));

		QuadraticSplinePath leftRingsToGoalDrop("Left Rings to Goal Drop");

		leftRingsToGoalDrop.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, 0)));
		leftRingsToGoalDrop.addPoint(SplinePoint(Point(60, 73), Vector(2, -M_PI_4)));
		leftRingsToGoalDrop.addPoint(SplinePoint(Point(75, 107), Vector(10, 0)));

		leftRingsToGoalDropIndex = purePursuit->addPath(leftRingsToGoalDrop.getPath(0.01));

		QuadraticSplinePath enterHomeZoneToRightHomeZone;

		enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(75, 95), Vector(10, 0)));
		enterHomeZoneToRightHomeZone.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, -M_PI_2)));

		enterHomeZoneToRightHomeZoneIndex = purePursuit->addPath(enterHomeZoneToRightHomeZone.getPath(0.01));

		QuadraticSplinePath rightHomeZoneToFarRightAllianceGoal;

		rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(129.2, 105.7), Vector(10, M_PI_2)));
		rightHomeZoneToFarRightAllianceGoal.addPoint(SplinePoint(Point(103, 125), Vector(10, M_PI_4)));

		rightHomeZoneToFarRightAllianceGoalIndex = purePursuit->addPath(rightHomeZoneToFarRightAllianceGoal.getPath(0.01));

		QuadraticSplinePath farRightAllianceGoalToNearPreloads;

		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(103, 125), Vector(40, M_PI_4)));
		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(129.2, 70.3), Vector(20, -M_PI)));
		farRightAllianceGoalToNearPreloads.addPoint(SplinePoint(Point(117.5, 5), Vector(10, 0)));

		farRightAllianceGoalToNearPreloadsIndex = purePursuit->addPath(farRightAllianceGoalToNearPreloads.getPath(0.01));

		SplinePath nearPlatformAlign;

		nearPlatformAlign.addPoint(117.5, 5);
		nearPlatformAlign.addPoint(117.5, 11.4);
		nearPlatformAlign.addPoint(130, 11.4);

		nearPlatformAlignIndex = purePursuit->addPath(nearPlatformAlign.getPath(0.01));

		QuadraticSplinePath leftHomeZoneToPlatform;

		leftHomeZoneToPlatform.addPoint(SplinePoint(Point(130, 11.4), Vector(10, -M_PI_2)));
		leftHomeZoneToPlatform.addPoint(SplinePoint(Point(80, 11.4), Vector(10, -M_PI_2)));

		leftHomeZoneToPlatformIndex = purePursuit->addPath(leftHomeZoneToPlatform.getPath(0.01));

		//!SECTION

		// SECTION Right Steal Right

		QuadraticSplinePath rightHomeZoneToRightNeutral("Right Home Zone to Right Neutral");

		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(105.7, 16), Vector(10, 0)));
		rightHomeZoneToRightNeutral.addPoint(SplinePoint(Point(105.7, 58), Vector(10, 0)));

		rightHomeZoneToRightNeutralIndex = purePursuit->addPath(rightHomeZoneToRightNeutral.getPath(0.01));

		QuadraticSplinePath rightNeutralToRightAllianceGoal;

		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(105.7, 58), Vector(20, 0)));
		rightNeutralToRightAllianceGoal.addPoint(SplinePoint(Point(125, 37), Vector(10, M_PI_4)));

		rightNeutralToRightAllianceGoalIndex = purePursuit->addPath(rightNeutralToRightAllianceGoal.getPath(0.01));

		QuadraticSplinePath rightAllianceGoalToRightRings;

		rightAllianceGoalToRightRings.addPoint(SplinePoint(Point(125, 37), Vector(10, M_PI_4)));
		rightAllianceGoalToRightRings.addPoint(SplinePoint(Point(117.5, 70), Vector(10, M_PI_4)));

		rightAllianceGoalToRightRingsIndex = purePursuit->addPath(rightAllianceGoalToRightRings.getPath(0.01));

		QuadraticSplinePath rightRingsToRightHomeZone;

		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(117.5, 70), Vector(10, M_PI_4)));
		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(90, 45), Vector(10, 0)));

		rightRingsToRightHomeZoneIndex = purePursuit->addPath(rightRingsToRightHomeZone.getPath(0.01));

		//!SECTION

		// SECTION Left Steal Left

		Path forward;

		forward.addPoint(0, 16);
		forward.addPoint(0, 60);

		forwardIndex = purePursuit->addPath(forward);

		Path backward;

		backward.addPoint(0, 60);
		backward.addPoint(0, 24);

		backwardIndex = purePursuit->addPath(backward);

		// !SECTION

		printf("Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
