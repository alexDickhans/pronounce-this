#include "purePursuit.hpp"

namespace Pronounce {
    PurePursuit::PurePursuit() {
    }

    PurePursuit::PurePursuit(double lookahead) {
        this->lookahead = lookahead;
    }
    PurePursuit::PurePursuit(OmniDrivetrain* drivetrain) {
        this->drivetrain = drivetrain;
        this->lookahead = 0;
    }
    PurePursuit::PurePursuit(OmniDrivetrain* drivetrain, double lookahead) {
        this->drivetrain = drivetrain;
        this->lookahead = lookahead;

    }

    void PurePursuit::update() {
        odometry->update();
        if (!enabled)
            return;

        if (!following) {
            drivetrain->setDriveVectorVelocity(Vector(0.0, 0.0));
            return;
        }

        if (paths.size() == 0 || currentPath == -1 || (paths.size() - 1 < currentPath))
            return;

        // Set position and path variables
        Path path = paths.at(currentPath);
        std::vector<Point> pathVector = path.getPath();
        Position* currentPosition = odometry->getPosition();
        Point currentPoint = Point(currentPosition->getX(), currentPosition->getY());

        // Returns if robot is close to target to prevent 
        if (pathVector.at(pathVector.size() - 1).distance(currentPoint) < stopDistance) {
            return;
        }

        // Get lookahead point and vector from robot
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, this->currentProfile.getLookaheadDistance());
        Vector lookaheadVector = Vector(&currentPoint, &lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() + currentPosition->getTheta());

        if (lookaheadVector.getMagnitude() > normalizeDistance) {
            lookaheadVector.setMagnitude(normalizeDistance);
        }

        // Normalize vectors to make dot product cleaner.
        Vector normalizedLastLookaheadVector = lastLookaheadVector;
        normalizedLastLookaheadVector.normalize();
        Vector normalizedLookaheadVector = lookaheadVector;
        normalizedLookaheadVector.normalize();

        // Get the dot product of how similar the last two vectors are
        double dotProduct = normalizedLookaheadVector.dot(normalizedLastLookaheadVector) - 1;
        dotProduct = ((dotProduct * curvatureMultiplier) / 2) + 1;

        lastLookaheadVector = lookaheadVector;
        
        // Set the drive vector

        // Map the magnitude to the distance from the lookahead distance to make sure that the robot's
        // PID controller behaves the same for different lookahead paths
        double magnitude = lookaheadVector.getMagnitude();
        double mappedMagnitude = map(magnitude, 0, currentProfile.getLookaheadDistance(), 0, normalizeDistance);
    
        currentProfile.getLateralPid()->setPosition(0);
        currentProfile.getLateralPid()->setTarget(mappedMagnitude);

        double lateralPower = currentProfile.getLateralPid()->update();
        Vector moveVector = Vector(lateralPower, lookaheadVector.getAngle());

        currentProfile.getTurnPid()->setPosition(currentPosition->getTheta());
        currentProfile.getTurnPid()->setTarget(turnTarget);

        double turnPower = currentProfile.getTurnPid()->update();

        drivetrain->setDriveVectorVelocity(moveVector.scale(dotProduct), turnPower);
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
