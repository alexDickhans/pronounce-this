#pragma once

#include "utils/pose2d.hpp"
#include "hardware/constants.hpp"

namespace PathPlanner {
	typedef struct MotionProfilePoint_ {
		Pronounce::Pose2D targetPose;
		QLength targetDistance;
		QSpeed targetSpeed;
		QCurvature targetCurvature;
		double targetT;
	} MotionProfilePoint;

	class AbstractMotionProfile {
	protected:
		Pronounce::ProfileConstraints defaultProfileConstraints;
		QSpeed maxSpeed;
		QLength trackWidth = Constants::trackWidth;
		std::vector<std::pair<double, std::string>> commands;
	public:
		AbstractMotionProfile() = default;

		explicit AbstractMotionProfile(const Pronounce::ProfileConstraints &defaultProfileConstraints)
				: defaultProfileConstraints(defaultProfileConstraints) {}

		virtual MotionProfilePoint update(QTime t) { return {}; }

		virtual QTime getDuration() { return {}; }

		std::vector<std::pair<double, std::string>> getCommands() { return commands; }

		~AbstractMotionProfile() = default;
	};
}