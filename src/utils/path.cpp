#include "path.hpp"

#include <iostream>

namespace Pronounce {
    Point Path::getLookAheadPoint(Point currentPosition, QLength lookaheadDistance) {
        Point lookaheadPoint = Point();
        bool pointFound = false;
		
        if (path.at(path.size() - 1).distance(currentPosition) <= lookaheadDistance && !continuePath) {
            return path.at(path.size() - 1);
        }

        for (int i = 1; i < path.size(); i++) {
            Point pathStart = path.at(i - 1);
            Point pathEnd = path.at(i);

            Vector d = Vector(pathStart, pathEnd);
            Vector f = Vector(currentPosition, pathStart);

            double a = d.dot(d);
            double b = 2 * f.dot(d);
            double c = f.dot(f) - (lookaheadDistance * lookaheadDistance).getValue();
            double discriminant = (b * b) - (4 * a * c);

            if (discriminant < 0) {

            }
            else {
                discriminant = sqrt(discriminant);
                double t1 = (-b - discriminant) / (2 * a);
                double t2 = (-b + discriminant) / (2 * a);

				if ((this->continuePath && i == path.size() - 1 && (t1 >= 1 || t2 >= 1))) {
				}

                if (0 <= t1 && t1 <= 1 && 0 <= t2 && t2 <= 1 || (this->continuePath && i == path.size() - 1 && (t1 >= 1 || t2 >= 1))) {
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
					break;
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
            return this->getClosestPoint(currentPosition);
        }

        return lookaheadPoint;
    }

    Path::Path() : Path("") {
    }

	Path::Path(std::string name) {
		this->name = name;
    }

    Point Path::getClosestPoint(Point currentPosition) {
        Point closestPoint;
        QLength closestDistance = (double) INT32_MAX;
		double closestT = INT32_MAX;

		double totalT = 0;

        // Returns the largest item in list
        // If two items are the same distance apart, will return first one
        for (int i = 1; i < path.size(); i++) {
            Point lastPoint = path.at(i - 1);
            Point thisPoint = path.at(i);

            Vector thisMinusLast(thisPoint, lastPoint);
            Vector positionMinusLast(currentPosition, lastPoint);

            // https://diego.assencio.com/?index=ec3d5dfdfc0b6a0d147a656f0af332bd
            double t = positionMinusLast.dot(thisMinusLast) / thisMinusLast.dot(thisMinusLast);

            if (0 < t && t < 1) {
                lastPoint += Vector(lastPoint, thisPoint).scale(t).getCartesian();
            } else if (t > 1) {
                lastPoint = thisPoint;
            } else if (t < 0) {
                lastPoint = lastPoint;
            }

            QLength distance = lastPoint.distance(currentPosition);
            if (distance < closestDistance) {
                closestDistance = distance;
                closestPoint = lastPoint;
				closestT = totalT + clamp(t, 0.0, 1.0);
            }

			totalT += 1;
        }

        return closestPoint;
    }

    Path::~Path() {
    }
} // namespace Pronounce
