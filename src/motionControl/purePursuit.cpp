#include "purePursuit.hpp"

namespace Pronounce {
    PurePursuit::PurePursuit() {
		this->path = Path();
		this->odometry = new ContinuousOdometry();
    }

    PurePursuit::PurePursuit(QLength lookahead) : PurePursuit() {
        this->currentProfile.lookaheadDistance = lookahead;
    }

	PurePursuit::PurePursuit(ContinuousOdometry* odometry, QLength lookahead) {
		this->path = Path();
		this->odometry = odometry;
		this->currentProfile.lookaheadDistance = lookahead;
	}

	PurePursuitPointData PurePursuit::updatePointData() {

        // Set position and path variables
        std::vector<Point> pathVector = path.getPath();
        Pose2D* currentPose = odometry->getPose();
        Point currentPoint = Point(currentPose->getX(), currentPose->getY());
		QLength lookahead = this->currentProfile.lookaheadDistance;

        // Get lookahead point and vector from robot
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, lookahead);
        Vector lookaheadVector = Vector(&currentPoint, &lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() + currentPose->getAngle());

        // Map the magnitude to the distance from the lookahead distance to make sure that the robot's
        // PID controller behaves the same for different lookahead paths
        QLength magnitude = lookaheadVector.getMagnitude();
        double mappedMagnitude = map(magnitude.getValue(), 0.0, lookahead.getValue(), 0, 1.0);
		Vector normalizedLookaheadVector = Vector(mappedMagnitude, lookaheadVector.getAngle());

		Vector robotRelativeLookaheadVector(&currentPoint, &lookaheadPoint);
		
		robotRelativeLookaheadVector.setAngle(robotRelativeLookaheadVector.getAngle() + currentPose->getAngle());

		QLength xDistance = robotRelativeLookaheadVector.getCartesian().getX();
		double signedCurvature = (2.0 * xDistance).getValue() / pow(lookaheadVector.getMagnitude().getValue(), 2);

		QLength distanceFromEnd = path.distanceFromEnd(currentPoint);

		this->pointData.lookaheadPoint = lookaheadPoint;
		this->pointData.lookaheadVector = lookaheadVector;
		this->pointData.localLookaheadVector = robotRelativeLookaheadVector;
		this->pointData.normalizedLookaheadVector = normalizedLookaheadVector;
		this->pointData.curvature = signedCurvature;
		this->pointData.distanceFromEnd = distanceFromEnd;
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
