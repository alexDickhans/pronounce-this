#include "purePursuit.hpp"

namespace Pronounce {
    PurePursuit::PurePursuit() {
		this->path = Path();
		this->odometry = new Odometry();
		this->currentProfile.lateralPid = new PID();
		this->currentProfile.orientationPid = new PID();
    }

    PurePursuit::PurePursuit(double lookahead) : PurePursuit() {
        this->currentProfile.lookaheadDistance = lookahead;
    }

	PurePursuit::PurePursuit(Odometry* odometry, double lookahead) {
		this->path = Path();
		this->odometry = odometry;
		this->currentProfile.lookaheadDistance = lookahead;
		this->currentProfile.lateralPid = new PID();
		this->currentProfile.orientationPid = new PID();
	}

	void PurePursuit::updatePointData() {

        // Set position and path variables
        std::vector<Point> pathVector = path.getPath();
        Position* currentPosition = odometry->getPosition();
        Point currentPoint = Point(currentPosition->getX(), currentPosition->getY());
		double lookahead = this->currentProfile.lookaheadDistance;

        // Returns if robot is close to target to prevent little wiggles
        if (this->isDone()) {
            return;
        }

        // Get lookahead point and vector from robot
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, lookahead);
        Vector lookaheadVector = Vector(&currentPoint, &lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() + currentPosition->getTheta());

        // Map the magnitude to the distance from the lookahead distance to make sure that the robot's
        // PID controller behaves the same for different lookahead paths
        double magnitude = lookaheadVector.getMagnitude();
        double mappedMagnitude = clamp(map(magnitude, 0, lookahead, 0, normalizeDistance), 0.0, normalizeDistance);
		Vector normalizedLookaheadVector = Vector(mappedMagnitude, lookaheadVector.getAngle());

		Vector robotRelativeLookaheadVector(&currentPoint, &lookaheadPoint);
		
		robotRelativeLookaheadVector.setAngle(robotRelativeLookaheadVector.getAngle() + currentPosition->getTheta());

		double xDistance = robotRelativeLookaheadVector.getCartesian().getX();
		double signedCurvature = (2 * xDistance) / pow(lookaheadVector.getMagnitude(), 2);

		double distanceFromEnd = path.distanceFromEnd(currentPoint);

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
