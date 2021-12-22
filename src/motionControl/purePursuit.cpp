#include "purePursuit.hpp"

namespace Pronounce {
    PurePursuit::PurePursuit() {
        this->setPurePursuitProfileManager(PurePursuitProfileManager());
    }

    PurePursuit::PurePursuit(double lookahead) : PurePursuit() {
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
            return;
        }

        // Get lookahead point and vector from robot
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, lookahead);
        Vector lookaheadVector = Vector(&currentPoint, &lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() + currentPosition->getTheta());

        // Normalize vectors to make dot product cleaner.
        Vector normalizedLookaheadVector = lookaheadVector;
        normalizedLookaheadVector.normalize();
        
        // Set the drive vector

        // Map the magnitude to the distance from the lookahead distance to make sure that the robot's
        // PID controller behaves the same for different lookahead paths
        double magnitude = lookaheadVector.getMagnitude();
        double mappedMagnitude = std::clamp(map(magnitude, 0, lookahead, 0, normalizeDistance), 0.0, normalizeDistance);
		Vector normalizedLookaheadVector = Vector(mappedMagnitude, lookaheadVector.getAngle());

		// Calculate curvature
		double a = -tan(currentPosition->getTheta());
		double b = 1;
		double c = tan(currentPosition->getTheta());

		double curvature = abs((a * lookaheadVector.getCartesian().getX()) + (b * lookaheadVector.getCartesian().getY()) + c) / sqrt(pow(a, 2) + pow(b, 2));
		double side = signum_c((sin(currentPosition->getTheta()) * lookaheadVector.getCartesian().getX()) - (cos(currentPosition->getTheta()) * lookaheadVector.getCartesian().getY()));
		double signedCurvature = curvature * side;

		this->pointData.lookaheadPoint = lookaheadPoint;
		this->pointData.lookaheadVector = lookaheadVector;
		this->pointData.normalizedLookaheadVector = normalizedLookaheadVector;
		this->pointData.curvature = signedCurvature;
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
