#pragma once

#include "utils/pose2d.hpp"
#include "hardware/constants.hpp"

namespace PathPlanner {
	typedef struct MotionProfilePoint_ {
		Pronounce::Pose2D targetPose = {};
		QLength targetDistance = 0.0;
		QVelocity targetSpeed = 0.0;
		QCurvature targetCurvature = 0.0;
		double targetT = 0.0;
	} MotionProfilePoint;

	class AbstractMotionProfile {
	protected:
		std::vector<std::pair<double, std::string>> commands;
	public:
		AbstractMotionProfile() = default;

		void processCommands(const Json& jsonCommands) {
			for (const auto& command : jsonCommands.array_items()) {
				this->commands.emplace_back(
						command["t"].number_value(),
						command["name"].string_value());
			}
		}

		[[nodiscard]] virtual MotionProfilePoint update(const QTime t) const { return {}; }

		[[nodiscard]] virtual QTime getDuration() const { return {}; }

		std::vector<std::pair<double, std::string>> getCommands() { return commands; }

		~AbstractMotionProfile() = default;
	};
}