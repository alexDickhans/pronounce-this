#pragma once

#include <utility>

#include "locolib/sensor/deadWheel/deadWheel.hpp"
#include "../orientationSource/orientationSource.hpp"

namespace Loco {
    typedef Eigen::Vector3d Pose2d;

    class TwoWheelOdometry {
    private:
        DeadWheel& horizontalWheel, &verticalWheel;
        OrientationSource& orientationSource;

		std::uniform_real_distribution<double> tickRandomness{-1, 1};
		std::normal_distribution<double> normalDistribution{0.0, 1.0};
		std::default_random_engine engine;

        Pose2d currentPose;

		Angle angleStd;
    public:
        TwoWheelOdometry(DeadWheel& horizontalWheel,
                         DeadWheel& verticalWheel,
                         OrientationSource& orientationSource,
						 Angle angleStd,
                         Pose2d startPose = Pose2d()) :
                         horizontalWheel(horizontalWheel),
                         verticalWheel(verticalWheel),
                         orientationSource(orientationSource),
                         angleStd(angleStd) {
            currentPose = std::move(startPose);
        }

        void update() {

			verticalWheel.update();
			horizontalWheel.update();

	        Angle angle = orientationSource.getAngle();

            Eigen::Vector3d localTranslation = Eigen::Vector3d({verticalWheel.getDelta().getValue(), horizontalWheel.getDelta().getValue(), 0.0});

            Eigen::Vector3d globalTranslation = Eigen::Matrix3d({
                {cos(-angle), sin(-angle), 0.0},
                {sin(-angle), cos(-angle), 0.0},
                {0.0, 0.0, 0.0}}) *
                        localTranslation;

            currentPose += globalTranslation;
            currentPose[2] = angle.getValue();
        }

		Pose2d predictDelta(double error = 0.02) {
			Angle angle = orientationSource.getAngle();

			Eigen::Vector3d localTranslation =
					Eigen::Vector3d({
		                horizontalWheel.getDelta().getValue(),
						verticalWheel.getDelta().getValue(),
						0.0});

			Eigen::Vector3d odomTickRandomness = {
					tickRandomness(engine) * verticalWheel.getTickLength().getValue(),
					tickRandomness(engine) * horizontalWheel.getTickLength().getValue(),
					0.0};

			localTranslation = localTranslation + odomTickRandomness;

			Eigen::Vector3d randomSpread = {
					localTranslation[0] * normalDistribution(engine) * error,
					localTranslation[1] * normalDistribution(engine) * error,
					angleStd.getValue() * normalDistribution(engine)};

			localTranslation = localTranslation + randomSpread;

			Eigen::Vector3d globalTranslation =
					Eigen::Matrix3d({
						{cos(-angle), -sin(-angle), 0.0},
						{sin(-angle), -cos(-angle), 0.0},
						{0.0, 0.0, 1.0}}) *
					localTranslation;

			return globalTranslation;
		}

        void reset(Pose2d newPose) {
            this->currentPose = std::move(newPose);
        }

        Pose2d getPose() {
            return currentPose;
        }
    };
}