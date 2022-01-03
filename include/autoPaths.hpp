#pragma once

#include "utils/pointUtil.hpp"
#include "utils/path.hpp"
#include "utils/splinePath.hpp"
#include "motionControl/purePursuit.hpp"
#include "utils/purePursuitProfile.hpp"
#include "utils/purePursuitProfileManager.hpp"

namespace Pronounce {

	// Test path
	int testPathIndex;

	// Right steal right
	int rightHomeToGoalNeutralIndex;
	int rightNeutralToMidNeutralIndex;
	int midNeutralToRightAllianceIndex;
	int midNeutralToMidHomeZoneIndex;
	int rightNeutralToRightHomeIndex;

	// Right awp right
	int farRightHomeZoneToRightAllianceIndex;
	int rightAllianceToRightHomeZoneIndex;

	// Left steal left
	int leftAllianceToLeftNeutralIndex;
	int leftNeutralToMidNeutralIndex;
	int midNeutralToLeftHomeZoneIndex;

	// Skills
	int rightNeutralToFarPlatformIndex;
	int farPlatformToNearPlatformIndex;
	int nearPlatformViaLeftNeutralToFarPlatformIndex;
	int nearPlatformToMidIndex;

	void autoPaths(PurePursuit purePursuit) {
		// Default pure pursuit profile
		PurePursuitProfile defaultProfile(new PID(20, 0.0, 2.0), new PID(60.0, 0.0, 5.0), 10.0);
		purePursuit.getPurePursuitProfileManager().setDefaultProfile(defaultProfile);

		// Test path
		Path testPath = Path();

		testPath.addPoint(0, 0);
		testPath.addPoint(24, 0);
		testPath.addPoint(24, 24);
		testPath.addPoint(24, 48);
		testPath.addPoint(-24, 48);
		testPath.addPoint(0, 24);

		testPathIndex = purePursuit.addPath(testPath);

		Path rightNeutralToRightHomeZone;

		rightNeutralToRightHomeZone.addPoint(105.7, 60);
		rightNeutralToRightHomeZone.addPoint(105.7, 16);

		rightNeutralToRightHomeIndex = purePursuit.addPath(rightNeutralToRightHomeZone);

		// Right Steal Right
		Path rightHomeToGoalNeutral;

		rightHomeToGoalNeutral.addPoint(105.7, 16);
		rightHomeToGoalNeutral.addPoint(105.7, 61);

		rightHomeToGoalNeutralIndex = purePursuit.addPath(rightHomeToGoalNeutral);

		Path rightNeutralToMidNeutral;

		rightNeutralToMidNeutral.addPoint(105.7, 62);
		rightNeutralToMidNeutral.addPoint(75.3, 40);
		rightNeutralToMidNeutral.addPoint(60.3, 65);

		rightNeutralToMidNeutralIndex = purePursuit.addPath(rightNeutralToMidNeutral);

		Path midNeutralToRightAlliance;

		midNeutralToRightAlliance.addPoint(70.3, 65);
		midNeutralToRightAlliance.addPoint(120.1, 28);

		midNeutralToRightAllianceIndex = purePursuit.addPath(midNeutralToRightAlliance);

		Path midNeutralToMidHomeZone;

		midNeutralToMidHomeZone.addPoint(70.3, 70.3);
		midNeutralToMidHomeZone.addPoint(70.3, 36);

		midNeutralToMidHomeZoneIndex = purePursuit.addPath(midNeutralToMidHomeZone);

		Path farRightHomeZoneToRightAlliance;

		farRightHomeZoneToRightAlliance.addPoint(127.9, 16);
		farRightHomeZoneToRightAlliance.addPoint(127.9, 24);

		farRightHomeZoneToRightAllianceIndex = purePursuit.addPath(farRightHomeZoneToRightAlliance);

		Path rightAllianceToRightHomeZone;

		rightAllianceToRightHomeZone.addPoint(127.9, 24);
		rightAllianceToRightHomeZone.addPoint(105.7, 16);

		rightAllianceToRightHomeZoneIndex = purePursuit.addPath(rightAllianceToRightHomeZone);

		Path leftAllianceToLeftNeutral;

		leftAllianceToLeftNeutral.addPoint(29, 11.4);
		leftAllianceToLeftNeutral.addPoint(32, 67);

		leftAllianceToLeftNeutralIndex = purePursuit.addPath(leftAllianceToLeftNeutral);

		Path leftNeutralToMidNeutral;

		leftNeutralToMidNeutral.addPoint(32, 67);
		leftNeutralToMidNeutral.addPoint(65.3, 40);
		leftNeutralToMidNeutral.addPoint(70.3, 65);

		leftNeutralToMidNeutralIndex = purePursuit.addPath(leftNeutralToMidNeutral);

		// mid neutral to mid home zone

		Path rightNeutralToFarPlatform;

		rightNeutralToFarPlatform.addPoint(105.7, 61);
		rightNeutralToFarPlatform.addPoint(75, 76.5);
		rightNeutralToFarPlatform.addPoint(75, 100);
		rightNeutralToFarPlatform.addPoint(60.3, 115);

		rightNeutralToFarPlatformIndex = purePursuit.addPath(rightNeutralToFarPlatform);

		Path farPlatformToNearPlatform;

		farPlatformToNearPlatform.addPoint(70.3, 107);
		farPlatformToNearPlatform.addPoint(60, 70.3);
		farPlatformToNearPlatform.addPoint(58.6, 64.1);
		farPlatformToNearPlatform.addPoint(58.6, 45);
		farPlatformToNearPlatform.addPoint(70.3, 30.7);

		farPlatformToNearPlatformIndex = purePursuit.addPath(farPlatformToNearPlatform);

		Path nearPlatformViaLeftNeutralToFarPlatform;

		nearPlatformViaLeftNeutralToFarPlatform.addPoint(70.3, 30.7);
		nearPlatformViaLeftNeutralToFarPlatform.addPoint(35, 61);
		nearPlatformViaLeftNeutralToFarPlatform.addPoint(70.3, 115);

		nearPlatformViaLeftNeutralToFarPlatformIndex = purePursuit.addPath(nearPlatformViaLeftNeutralToFarPlatform);

		Path nearPlatformToMid;

		nearPlatformToMid.addPoint(70.3, 115);
		nearPlatformToMid.addPoint(70.3, 70.3);

		nearPlatformToMidIndex = purePursuit.addPath(nearPlatformToMid);
	}

} // Namespace Prononce
