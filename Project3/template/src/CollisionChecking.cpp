///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Haoran Liang(hl74) & Hao Ding(hd25)
//////////////////////////////////////

#include "CollisionChecking.h"
#include "Util.h"

// Intersect the point (x,y) with the set of rectangles. If the point lies outside of all obstacles, return true.
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

// Add any custom debug / development code here. This code will be executed
// instead of the statistics checker (Project2.cpp). Any code submitted here
// MUST compile, but will not be graded.
void debugMode(const std::vector<Robot> & robots, const std::vector<Rectangle> & obstacles,
               const std::vector<bool> & valid)
{
	Robot robot = robots[0];
		double Pi = acos(-1);
	//    cout << sin(0.5 * Pi) << endl;
	//    cout << isValidSquare(robot.x, robot.y, robot.theta, robot.length, obstacles) << endl;
		cout << isValidSquare(robot.x, robot.y, robot.theta, robot.length, obstacles) << endl;
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
