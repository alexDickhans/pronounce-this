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
	int enterHomeZoneToRightHomeZoneIndex;

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

		QuadraticSplinePath leftAllianceToRightHomeZone = QuadraticSplinePath();

		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(30.0, 11.4), Vector(0, M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(15.2, 35.0), Vector(0, 0.0)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(46.4, 46.4), Vector(0, M_PI_2)));
		leftAllianceToRightHomeZone.addPoint(SplinePoint(Point(80, 46.4), Vector(0, M_PI_2)));

		leftAllianceToRightHomeZoneIndex = purePursuit->addPath(leftAllianceToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightHomeZoneToRightAlliance = QuadraticSplinePath();

		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(80, 46.4), Vector(15.0, -M_PI_2 - M_PI_4)));
		rightHomeZoneToRightAlliance.addPoint(SplinePoint(Point(125.0, 30), Vector(15.0, -M_PI_2 - M_PI_4)));

		rightHomeZoneToRightAllianceIndex = purePursuit->addPath(rightHomeZoneToRightAlliance.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightHomeZone = QuadraticSplinePath();

		rightAllianceToRightHomeZone.addPoint(SplinePoint(Point(130.0, 30), Vector(15.0, -M_PI_2 - M_PI_4)));
		rightAllianceToRightHomeZone.addPoint(SplinePoint(Point(80, 30), Vector(15.0, -M_PI_2 - M_PI_4)));

		rightAllianceToRightHomeZoneIndex = purePursuit->addPath(rightAllianceToRightHomeZone.getPath(0.1));

		QuadraticSplinePath rightAllianceToRightRings = QuadraticSplinePath();

		rightAllianceToRightRings.addPoint(SplinePoint(Point(130.0, 26), Vector(15.0, -M_PI_2 - M_PI_4)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(117.5, 46.8), Vector(0.0, 0.0)));
		rightAllianceToRightRings.addPoint(SplinePoint(Point(117.5, 70.3), Vector(20.0, 0.0)));

		rightAllianceToRightRingsIndex = purePursuit->addPath(rightAllianceToRightRings.getPath(0.1));

		QuadraticSplinePath rightRingsToRightHomeZone = QuadraticSplinePath();

		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(117.5, 70.3), Vector(20.0, M_PI)));
		rightRingsToRightHomeZone.addPoint(SplinePoint(Point(85, 36), Vector(20.0, M_PI_2)));

		rightRingsToRightHomeZoneIndex = purePursuit->addPath(rightRingsToRightHomeZone.getPath(0.1));

		//SECTION Skills

		// Left home zone to left neutral goal
		QuadraticSplinePath leftHomeZoneToLeftNeutralGoal;

		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(23.2, 11.4), Vector(25, M_PI_4 * 1.5)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(48, 46.8), Vector(15, 0)));
		leftHomeZoneToLeftNeutralGoal.addPoint(SplinePoint(Point(35, 78), Vector(35, M_PI_4 * 0.75)));

		leftHomeZoneToLeftNeutralGoalIndex = purePursuit->addPath(leftHomeZoneToLeftNeutralGoal.getPath(0.01));

		QuadraticSplinePath leftNeutralGoalToFarHomeZone;

		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(35, 66), Vector(30, -M_PI_4)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70.3, 93.9), Vector(30, 0)));
		leftNeutralGoalToFarHomeZone.addPoint(SplinePoint(Point(70.3, 107), Vector(20, 0)));

		leftNeutralGoalToFarHomeZoneIndex = purePursuit->addPath(leftNeutralGoalToFarHomeZone.getPath(0.01));

		QuadraticSplinePath farHomeZoneToMidNeutralGoal;

		farHomeZoneToMidNeutralGoal.addPoint(SplinePoint(Point(70.3, 107), Vector(20, 0)));
		farHomeZoneToMidNeutralGoal.addPoint(SplinePoint(Point(70.3, 75), Vector(30, 0)));

		farHomeZoneToMidNeutralGoalIndex = purePursuit->addPath(farHomeZoneToMidNeutralGoal.getPath(0.01));

		QuadraticSplinePath midNeutralGoalToPlatform;

		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(70.3, 75), Vector(30, 0)));
		midNeutralGoalToPlatform.addPoint(SplinePoint(Point(75, 107), Vector(30, 0)));

		midNeutralGoalToPlatformIndex = purePursuit->addPath(midNeutralGoalToPlatform.getPath(0.01));

		QuadraticSplinePath farPlatformToDropOffGoal;

		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(65, 107), Vector(5, 0)));
		farPlatformToDropOffGoal.addPoint(SplinePoint(Point(65, 80), Vector(5, 0)));

		farPlatformToDropOffGoalIndex = purePursuit->addPath(farPlatformToDropOffGoal.getPath(0.01));

		QuadraticSplinePath goalDropOffToFarLeftAlliance;

		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(65, 80), Vector(15, 0)));
		goalDropOffToFarLeftAlliance.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4)));

		goalDropOffToFarLeftAllianceIndex = purePursuit->addPath(goalDropOffToFarLeftAlliance.getPath(0.01));

		QuadraticSplinePath farLeftAllianceToLeftRings;

		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(18, 97), Vector(15, M_PI_4 + M_PI)));
		farLeftAllianceToLeftRings.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, M_PI)));

		farLeftAllianceToLeftRingsIndex = purePursuit->addPath(farLeftAllianceToLeftRings.getPath(0.01));

		QuadraticSplinePath leftRingsToGoalDrop;

		leftRingsToGoalDrop.addPoint(SplinePoint(Point(23.2, 70.3), Vector(15, 0)));
		leftRingsToGoalDrop.addPoint(SplinePoint(Point(60, 73), Vector(2, -M_PI_4)));
		leftRingsToGoalDrop.addPoint(SplinePoint(Point(75, 107), Vector(10, 0)));

		leftRingsToGoalDropIndex = purePursuit->addPath(leftRingsToGoalDrop.getPath(0.01));

		QuadraticSplinePath farPlatformToEnterHomezone;

		farPlatformToEnterHomezone.addPoint(SplinePoint(Point(75, 107), Vector(2, 0)));
		farPlatformToEnterHomezone.addPoint(SplinePoint(Point(75, 95), Vector(2, 0)));

		farPlatformToEnterHomezoneIndex = purePursuit->addPath(farPlatformToEnterHomezone.getPath(0.01));

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

		Path forward;

		forward.addPoint(0, 16);
		forward.addPoint(0, 60);

		forwardIndex = purePursuit->addPath(forward);

		Path backward;

		//!SECTION

		printf("Array size: %d\n", purePursuit->getPaths().size());
	}

} // Namespace Prononce
