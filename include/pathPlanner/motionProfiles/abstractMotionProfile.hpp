#pragma once

#include "utils/pose2d.hpp"
#include "hardware/constants.hpp"

namespace PathPlanner {
	typedef struct MotionProfilePoint_ {
		Pronounce::Pose2D targetPose = {};
		QLength targetDistance = 0.0;
		QSpeed targetSpeed = 0.0;
		QCurvature targetCurvature = 0.0;
		double targetT = 0.0;
		bool inverted = false;
	} MotionProfilePoint;

	class AbstractMotionProfile {
	protected:
		QVelocity maxSpeed = Constants::maxSpeed;
		QLength trackWidth = Constants::trackWidth;
		std::vector<std::pair<double, std::string>> commands;
	public:
		AbstractMotionProfile() = default;

		[[nodiscard]] virtual MotionProfilePoint update(QTime t) const { return {}; }

		[[nodiscard]] virtual QTime getDuration() const { return {}; }

		std::vector<std::pair<double, std::string>> getCommands() { return commands; }

		[[nodiscard]] const QVelocity &getMaxSpeed() const {
			return maxSpeed;
		}

		void setMaxSpeed(const QVelocity &maxSpeed) {
			AbstractMotionProfile::maxSpeed = maxSpeed;
		}

		[[nodiscard]] const QLength &getTrackWidth() const {
			return trackWidth;
		}

		void setTrackWidth(const QLength &trackWidth) {
			AbstractMotionProfile::trackWidth = trackWidth;
		}

		~AbstractMotionProfile() = default;
	};
}