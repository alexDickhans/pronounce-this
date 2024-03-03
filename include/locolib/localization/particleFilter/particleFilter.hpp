#pragma once

#include <vector>
#include "particle.hpp"
#include <random>
#include "units/units.hpp"
#include "locolib/prediction/deadWheel/twoWheelOdometry.hpp"
#include "locolib/sensor/sensorModel.hpp"
#include "locolib/utils/fieldModel.hpp"

namespace Loco {
	template<size_t size>
	class ParticleFilter {
	private:
		std::array<Particle, size> particles;
		std::default_random_engine generator;
		std::uniform_int_distribution<int> randomItemDistribution{0, size-1};
		std::uniform_real_distribution<double> resamplingDistribution{0, 1.0};
		std::normal_distribution<double> startingNormal{0.0, 1.0};
		std::uniform_real_distribution<double> startingUniform;
		OrientationSource& orientationSource;
		TwoWheelOdometry& posePredictor;
		Particle bestParticle;
		std::vector<SensorModel*> sensors{};
		double totalWeight = 0.0;
	public:
		ParticleFilter(OrientationSource& orientationSource, TwoWheelOdometry& posePredictor) : orientationSource(orientationSource), posePredictor(posePredictor) {
			this->initializeUniform(FieldModel::minPosition, FieldModel::maxPosition);
		}

		void addSensor(SensorModel* sensor) {
			sensors.emplace_back(sensor);
		}

		void initializeUniform(const Eigen::Vector2d& smallBound, const Eigen::Vector2d& largeBound) {
			Eigen::Vector2d boundDifference = (largeBound - smallBound) * 0.5;
			Eigen::Matrix2d multiplierMatrix = Eigen::Matrix2d({{boundDifference.x(), 0},
																{0, boundDifference.y()}});
			Eigen::Vector2d middlePoint = smallBound + boundDifference;

			for (size_t i = 0; i < size; i++) {
				auto randVector = middlePoint + multiplierMatrix * Eigen::Vector2d::Random();
				particles.at(i) = Eigen::Vector3d(randVector.x(), randVector.y(), orientationSource.getAngle().getValue());
			}
		}

		void initializeNormal(const Eigen::Vector2d& startingPosition, const Eigen::Matrix2d& covariance, Eigen::Vector2d lowerBound = FieldModel::minPosition, Eigen::Vector2d upperBound = FieldModel::maxPosition) {
			for (size_t i = 0; i < size; i++) {
				particles.at(i) = Particle(startingPosition + (covariance * Eigen::Vector2d(startingNormal(generator), startingNormal(generator))), orientationSource.getAngle());
				particles.at(i).clamp(lowerBound, upperBound);
			}
		}

		void initializeSpot(const Eigen::Vector3d& startingPosition) {
			for (auto &particle: particles) {
				particle = startingPosition;
			}
		}

		void setOrientation(Angle orientation) {
			for (auto &item: particles) {
				item.setAngle(orientation);
			}
		}

		void setOrientation() {
			this->setOrientation(orientationSource.getAngle());
		}

		void moveParticles(const std::function<Eigen::Vector3d(void)>& translationFunction) {
			for (auto &item: particles) {
				item.setState(item.getState() + translationFunction());
			}
		}

		void weightParticles() {
			double highestWeight = std::numeric_limits<double>::min();
			totalWeight = 0.0;

			for (auto &particle: particles) {
				double weight = 0.0;
				double weightModifiers = 0.0;

				for (auto &sensor: sensors) {
					weight += sensor->confidence(particle) * sensor->getWeight();
					weightModifiers += sensor->getWeight();
				}

				particle.setWeight(weight/weightModifiers);
				totalWeight += particle.getWeight();

				std::cout << "HIIW:" << weight << std::endl;
				if (particle.getWeight() > highestWeight) {
					highestWeight = weight;
					bestParticle = particle;
				}
			}
		}

		void resampleParticles() {

			auto oldParticles = particles;

			double startValue = resamplingDistribution(generator) * totalWeight / static_cast<double>(size);
			size_t currentI = 0;
			double endWeight = oldParticles.at(0).getWeight();

			for (size_t i = 0; i < size; i++) {
				double currentWeight = startValue + static_cast<double>(i) * totalWeight / static_cast<double>(size);

				while (endWeight < currentWeight) {
					currentI ++;
					endWeight += oldParticles.at(i).getWeight();
				}

				particles.at(i) = oldParticles.at(std::min(currentI, size-1));
			}
		}

		void clampParticles(const Eigen::Vector2d& lowerBound = FieldModel::minPosition, const Eigen::Vector2d& upperBound = FieldModel::maxPosition) {
			for (auto &item: particles) {
				item.clamp(lowerBound, upperBound);
			}
		}

		void update() {
			QTime timeDelta = 20_ms;

			posePredictor.update();

			this->setOrientation();

			this->moveParticles([&] () -> Eigen::Vector3d {return posePredictor.predictDelta(0.04);});

			if (sensors.empty())
				return;

			this->weightParticles();

			this->resampleParticles();

			this->clampParticles();
		}

		Particle getBestParticle() {
			return bestParticle;
		}

		Particle getRandomParticle() {
			return particles.at(randomItemDistribution(generator));
		}

		Particle getMeanParticle() {
			Eigen::Vector3d sum = {};

			for (const auto &particle: particles) {
				sum = sum + particle.getState();
			}

			sum /= size;

			return {sum};
		}
	};
}