#include "path.hpp"

#include <iostream>

namespace Pronounce {
    Point Path::getLookAheadPoint(Point currentPosition, double lookaheadDistance) {
        Point lookaheadPoint = Point();
        bool pointFound = false;

        if (path.at(path.size() - 1).distance(currentPosition) <= lookaheadDistance) {
            return path.at(path.size() - 1);
        }

        for (int i = 1; i < path.size(); i++) {
            Point pathStart = path.at(i - 1);
            Point pathEnd = path.at(i);

            Vector d = Vector(&pathStart, &pathEnd);
            Vector f = Vector(&currentPosition, &pathStart);

            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - (lookaheadDistance * lookaheadDistance);
            double discriminant = (b * b) - (4 * a * c);

            if (discriminant < 0) {

            } else {
                discriminant = sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

                if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1) {
                    if (t2 < t1) {
                        pointFound = true;
                        Vector resultVector = d.scale(t1);
                        Point tempLookaheadPoint = pathStart;
                        tempLookaheadPoint += resultVector.getCartesian();
                        lookaheadPoint = tempLookaheadPoint;
                    }
                    if (t1 < t2) {
                        pointFound = true;
                        Vector resultVector = d.scale(t2);
                        Point tempLookaheadPoint = pathStart;
                        tempLookaheadPoint += resultVector.getCartesian();
                        lookaheadPoint = tempLookaheadPoint;
                    }
                }
                if (0 <= t1 && t1 <= 1) {
                    pointFound = true;
                    Vector resultVector = d.scale(t1);
                    Point tempLookaheadPoint = pathStart;
                    tempLookaheadPoint += resultVector.getCartesian();
                    lookaheadPoint = tempLookaheadPoint;
                }
                if (0 <= t2 && t2 <= 1) {
                    pointFound = true;
                    Vector resultVector = d.scale(t2);
                    Point tempLookaheadPoint = pathStart;
                    tempLookaheadPoint += resultVector.getCartesian();
                    lookaheadPoint = tempLookaheadPoint;
                }
            }
        }

        if (!pointFound) {
            return getClosestPoint(currentPosition);
        }

        return lookaheadPoint;
    }

    Path::Path(/* args */) {
    }

    Point Path::getClosestPoint(Point currentPosition) {
        Point closestPoint;
        double closestDistance = 0;

        // Returns the largest item in list
        // If two items are the same distance apart, will return first one
        for (int i = 0; i < path.size(); i++) {
            double distance = path.at(i).distance(currentPosition);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPoint = path.at(i);
            }
        }

        return closestPoint;
    }

    Path::~Path() {
    }
} // namespace Pronounce