///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Haoran Liang (hl74) & Hao Ding (hd25)
//////////////////////////////////////

#include "CollisionChecking.h"
//#include "Util.h"

// TODO: Copy your implementation of Project 2!

bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for(int i = 0; i < obstacles.size(); i++) {
    	// x, y is the lower left corner
    	// xMax = x + width
    	// yMax = y + height
    	double a = obstacles[i].x;
    	double b = obstacles[i].y;
    	double width = obstacles[i].width;
    	double height = obstacles[i].height;

    	// xMin <= x <= xMax && yMin <= y <= yMax
    	double aMax = a + width;
    	double bMax = b + height;

    	if(((x >= a) && (x <= aMax)) && ((y >= b) && (y <= bMax))) {
    		return false;
    	}
    }
    return true;
}

// Intersect a circle with center (x,y) and given radius with the set of rectangles. If the circle lies outside of all
// obstacles, return true.
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles)
{
    // TODO: IMPLEMENT ME!!
    for(int i = 0; i < obstacles.size(); i++) {
    	double a = obstacles[i].x;
    	double b = obstacles[i].y;
    	double width = obstacles[i].width;
    	double height = obstacles[i].height;

    	// Check for ((xMin - r <= x <= xMax + r) && (yMin <= y <= yMax)) || ((xMin <= x <= xMax) && (yMin - r <= y <= yMax + r)) || ce(Euclidean norm) <= r
    	double aMax = a + width;
    	double bMax = b + height;
    	double radiusPower = pow(radius, 2);

    	if(((a - radius <= x && aMax + radius >= x) && (b <= y && bMax >= y)) ||
    		((a <= x && aMax >= x) && (b - radius <= y && bMax + radius >= y)) ||
    		(pow((a - x), 2) + pow((b - y), 2) <= radiusPower) ||	// lower left vertex
    		(pow((aMax - x), 2) + pow((bMax - y), 2) <= radiusPower) ||	// lower right vertex
    		(pow((a - x), 2) + pow((bMax - y), 2) <= radiusPower) || // upper left vertex
    		(pow((aMax - x), 2) + pow((b - y), 2) <= radiusPower))	// upper right vertex 
    	{
    		return false;
    	}
    }
    return true;
}

// Intersect a square with center at (x,y), orientation theta, and the given side length with the set of rectangles. If
// the square lies outside of all obstacles, return true.
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles)
{
	// TODO: IMPLEMENT ME!!

    double r = 1.0 * sideLength / sqrt(2);
    double Pi = acos(-1);
    vector<pair<double, double>> verticesOfSquare;
    for (int i = 0; i < 4; i ++) {
        verticesOfSquare.push_back(make_pair(x + r * cos((0.25 + i * 0.5) * Pi + theta), y + r * sin((0.25 + i * 0.5) * Pi + theta)));
    }

    for (Rectangle obstacle : obstacles) {

        // if the center of square is in obstacle, it must collide with obstacle, return false
        if (obstacle.x <= x && x <= obstacle.x + obstacle.width && obstacle.y <= y && y <= obstacle.y + obstacle.height) {
            return false;
        }

        vector<pair<double, double>> verticesOfObstacle;
        verticesOfObstacle.push_back(make_pair(obstacle.x, obstacle.y));
        verticesOfObstacle.push_back(make_pair(obstacle.x + obstacle.width, obstacle.y));
        verticesOfObstacle.push_back(make_pair(obstacle.x + obstacle.width, obstacle.y + obstacle.height));
        verticesOfObstacle.push_back(make_pair(obstacle.x, obstacle.y + obstacle.height));

        for (int i = 0 ; i < 4; i ++) {
            for (int j = 0; j < 4; j ++) {
                if (checkIntersection(verticesOfSquare[i], verticesOfSquare[(i + 1) % 4], verticesOfObstacle[j], verticesOfObstacle[(j + 1) % 4])) {
                    return false;
                }
            }
        }
    }

    return true;
}

/**
 * Compute the cross product of two vectors
 * @param vector1
 * @param vector2
 * @return cross product of vector1 and vector2
 */
double computeCrossProduct(pair<double, double> vector1, pair<double, double> vector2) {
    return vector1.first * vector2.second - vector1.second * vector2.first;
}

/**
 * Check whether 3 vertices are collinear
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @return whether 3 vertices are collinear
 */
bool isCollinear(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3) {
    pair<double, double> vector1 = make_pair(vertex1.first - vertex2.first, vertex1.second - vertex2.second);
    pair<double, double> vector2 = make_pair(vertex1.first - vertex3.first, vertex1.second - vertex3.second);

    if (computeCrossProduct(vector1, vector2) == 0) {
        return true;
    }

    return false;
}

/**
 * Check whether vertex3 is on segment [vertex1, vertex2]
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @return bool value
 */
bool isOnSegment(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3) {
    if (!isCollinear(vertex1, vertex2, vertex3)) {
        return false;
    }

    double minX = min(vertex1.first, vertex2.first);
    double maxX = max(vertex1.first, vertex2.first);
    double minY = min(vertex1.second, vertex2.second);
    double maxY = max(vertex1.second, vertex2.second);
    if (minX <= vertex3.first && vertex3.first <= maxX && minY <= vertex3.second && vertex3.second <= maxY) {
        return true;
    }

    return false;
}

/**
 * Check whether two segments [v1, v2] and [v3, v4] are parallel
 * Note: Collinear is not equal to parallel
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @param vertex4
 * @return whether these 2 segments are parallel
 */
bool isParallel(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3, pair<double, double> vertex4) {
    // handle collinear case
    if (isCollinear(vertex1, vertex2, vertex3) || isCollinear(vertex1, vertex2, vertex4)) {
        return false;
    }

    // vertical to x-axis
    if (vertex1.first == vertex2.first && vertex3.first == vertex4.first) {
        return true;
        // use slope of two segments to determine whether they are parallel
    } else if ((vertex1.first - vertex2.first) * (vertex3.second - vertex4.second) == (vertex1.second - vertex2.second) * (vertex3.first - vertex4.first)) {
        return true;
    }

    return false;
}

/**
 * Check whether segment [v1, v2] intersects with segment [v3, v4]
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @param vertex4
 * @return whether these two segments intersect with each other
 */
bool checkIntersection(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3, pair<double, double> vertex4) {
    // handle collinear case, disregarding many intersection and one intersection cases
    if (isCollinear(vertex1, vertex2, vertex3) && isCollinear(vertex1, vertex2, vertex4)) {
        return false;
    }

    // Not collinear and one vertex of a segment is on another segment, return true
    if (isOnSegment(vertex1, vertex2, vertex3)) return true;
    if (isOnSegment(vertex1, vertex2, vertex4)) return true;
    if (isOnSegment(vertex3, vertex4, vertex1)) return true;
    if (isOnSegment(vertex3, vertex4, vertex2)) return true;

    // handle parallel case
    if (isParallel(vertex1, vertex2, vertex3, vertex4)) {
        return false;
    }

    // if two segments intersects with each other, vertex3 and vertex4 should be on different side of segment [vertex1, vertex2],
    // then the cross product of (vector1, vector2) and (vector1, vector3) should have different signs
    pair<double, double> vector1 = make_pair(vertex1.first - vertex2.first, vertex1.second - vertex2.second);
    pair<double, double> vector2 = make_pair(vertex1.first - vertex3.first, vertex1.second - vertex3.second);
    pair<double, double> vector3 = make_pair(vertex1.first - vertex4.first, vertex1.second - vertex4.second);

    if (computeCrossProduct(vector1, vector2) * computeCrossProduct(vector1, vector3) >= 0) {
        return false;
    }

    // Also, if two segments intersects with each other, vertex1 and vertex2 should be on different side of segment [vertex3, vertex3],
    // then the cross product of (vector1, vector2) and (vector1, vector3) should have different signs
    vector1 = make_pair(vertex3.first - vertex4.first, vertex3.second - vertex4.second);
    vector2 = make_pair(vertex3.first - vertex1.first, vertex3.second - vertex1.second);
    vector3 = make_pair(vertex3.first - vertex2.first, vertex3.second - vertex2.second);

    if (computeCrossProduct(vector1, vector2) * computeCrossProduct(vector1, vector3) >= 0) {
        return false;
    }

    return true;
}


// Based on rotation transformation
// (x', y') = T * R(theta) * -T
// Compute point coordiante x after rotate around pivot point with angle theta
double pivotPointRotationX(double x, double y, double theta, double pivotX, double pivotY) {
	double rotatedX = cos(theta) * x - cos(theta) * pivotX - sin(theta) * y + sin(theta) * pivotY + pivotX;
	return rotatedX;
}

// Compute point coordiante y after rotate around pivot point with angle theta
double pivotPointRotationY(double x, double y, double theta, double pivotX, double pivotY) {
	double rotatedY = sin(theta) * x - sin(theta) * pivotX + cos(theta) * y - cos(theta) * pivotY + pivotY;
	return rotatedY;
}