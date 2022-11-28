#include "purePursuit.hpp"

namespace Pronounce {

	PurePursuit::PurePursuit(std::string name, ContinuousOdometry* odometry, PurePursuitProfile currentProfile, Path path) : Behavior(name) {
		this->path = path;
		this->odometry = odometry;
		this->currentProfile = currentProfile;
		this->currentProfile.velocityProfile.setDistance(this->path.distance());
		// this->currentProfile.velocityProfile.calculate(100);
	}

	PurePursuitPointData PurePursuit::updatePointData() {

        // Set position and path variables
        Pose2D currentPose = odometry->getPose();
        Point currentPoint = Point(currentPose.getX(), currentPose.getY());
		QLength lookahead = this->currentProfile.lookaheadDistance;

        // Get lookahead point and vector from robot
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, lookahead);
        Vector lookaheadVector = Vector(currentPoint, lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() + currentPose.getAngle());

        // Map the magnitude to the distance from the lookahead distance to make sure that the robot's
        // PID controller behaves the same for different lookahead paths
        QLength magnitude = lookaheadVector.getMagnitude();
        double mappedMagnitude = map(magnitude.getValue(), 0.0, lookahead.getValue(), 0, 1.0);
		Vector normalizedLookaheadVector = Vector(mappedMagnitude, lookaheadVector.getAngle());

		Vector robotRelativeLookaheadVector(currentPoint, lookaheadPoint);
		
		robotRelativeLookaheadVector.setAngle(robotRelativeLookaheadVector.getAngle() + currentPose.getAngle());

		QLength xDistance = robotRelativeLookaheadVector.getCartesian().getX();
		double signedCurvature = (2.0 * xDistance).getValue() / pow(lookaheadVector.getMagnitude().getValue(), 2);

		QLength distanceFromBeginning = path.distanceFromStart(currentPoint);
		QLength distanceFromEnd = path.distanceFromEnd(currentPoint);
		
		PurePursuitPointData pointData;

		pointData.lookaheadPoint = lookaheadPoint;
		pointData.lookaheadVector = lookaheadVector;
		pointData.localLookaheadVector = robotRelativeLookaheadVector;
		pointData.normalizedLookaheadVector = normalizedLookaheadVector;
		pointData.curvature = signedCurvature;
		pointData.distanceFromBeginning = distanceFromBeginning;
		pointData.distanceFromEnd = distanceFromEnd;

		return pointData;
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
