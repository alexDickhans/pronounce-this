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
        Point lookaheadPoint = path.getLookAheadPoint(currentPoint, lookahead);
        Vector lookaheadVector = Vector(&currentPoint, &lookaheadPoint);
        lookaheadVector.setAngle(lookaheadVector.getAngle() - currentPosition->getTheta());

        lateralPid->setPosition(0);
        lateralPid->setTarget(lookaheadVector.getMagnitude());

        double lateralPower = lateralPid->update();
        Vector moveVector = Vector(lateralPower, lookaheadVector.getAngle());

        anglePid->setPosition(currentPosition->getTheta());
        anglePid->setTarget(turnTarget);

        double turnPower = anglePid->update();

        drivetrain->setDriveVectorVelocity(moveVector, turnPower);
    }

    PurePursuit::~PurePursuit() {
    }
} // namespace Pronounce
