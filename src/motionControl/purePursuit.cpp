#include "purePursuit.hpp"

namespace Pronounce {
    PurePursuit::PurePursuit() {
        this->setPurePursuitProfileManager(PurePursuitProfileManager());
		this->paths = std::vector<Path>();
		this->odometry = new Odometry();
    }

    PurePursuit::PurePursuit(double lookahead) : PurePursuit() {
        this->lookahead = lookahead;
    }

	PurePursuit::PurePursuit(Odometry* odometry, double lookahead) {
		this->setPurePursuitProfileManager(PurePursuitProfileManager());
		this->paths = std::vector<Path>();
		this->odometry = odometry;
		this->lookahead = lookahead;
	}

	void PurePursuit::updatePointData() {

        // Set position and path variables
        Path path = paths.at(currentPath);
        std::vector<Point> pathVector = path.getPath();
        Position* currentPosition = odometry->getPosition();
        Point currentPoint = Point(currentPosition->getX(), currentPosition->getY());

        // Returns if robot is close to target to prevent little wiggles
        if (pathVector.at(pathVector.size() - 1).distance(currentPoint) < stopDistance) {
			this->atPoint = true;
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

		Vector robotRelativeLookaheadVector = lookaheadVector;
		
		robotRelativeLookaheadVector.rotate(-currentPosition->getTheta());

		double xDistance = robotRelativeLookaheadVector.getCartesian().getX();
		double signedCurvature = (2 * xDistance) / pow(lookaheadVector.getMagnitude(), 2);

		this->pointData.lookaheadPoint = lookaheadPoint;
		this->pointData.lookaheadVector = lookaheadVector;
		this->pointData.localLookaheadVector = robotRelativeLookaheadVector;
		this->pointData.normalizedLookaheadVector = normalizedLookaheadVector;
		this->pointData.curvature = signedCurvature;
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
