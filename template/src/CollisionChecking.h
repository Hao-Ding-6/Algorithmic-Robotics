#ifndef COLLISION_CHECKING_H_
#define COLLISION_CHECKING_H_

#include <vector>
#include <cmath>
#include <iostream>

using namespace std;

struct Rectangle
{
    // Coordinate of the lower left corner of the rectangle
    double x, y;
    // The width (x-axis extent) of the rectangle
    double width;
    // The height (y-axis extent) of the rectangle
    double height;
};

// Intersect the point (x,y) with the set of rectangles. If the point lies
// outside of all obstacles, return true.
bool isValidPoint(double x, double y, const std::vector<Rectangle> &obstacles);

// Intersect a circle with center (x,y) and given radius with the set of
// rectangles. If the circle lies outside of all obstacles, return true
bool isValidCircle(double x, double y, double radius, const std::vector<Rectangle> &obstacles);

// Intersect a square with center at (x,y), orientation theta, and the given
// side length with the set of rectangles. If the square lies outside of all
// obstacles, return true
bool isValidSquare(double x, double y, double theta, double sideLength, const std::vector<Rectangle> &obstacles);

/**
 * Compute the cross product of two vectors
 * @param vector1
 * @param vector2
 * @return cross product of vector1 and vector2
 */
double computeCrossProduct(pair<double, double> vector1, pair<double, double> vector2);

/**
 * Check whether 3 vertices are collinear
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @return whether 3 vertices are collinear
 */
bool isCollinear(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3);

/**
 * Check whether two segments [v1, v2] and [v3, v4] are parallel
 * Note: Collinear is not equal to parallel
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @param vertex4
 * @return whether these 2 segments are parallel
 */
bool isParallel(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3, pair<double, double> vertex4);

/**
 * Check whether segment [v1, v2] intersects with segment [v3, v4]
 * @param vertex1
 * @param vertex2
 * @param vertex3
 * @param vertex4
 * @return whether these two segments intersect with each other
 */
bool checkIntersection(pair<double, double> vertex1, pair<double, double> vertex2, pair<double, double> vertex3, pair<double, double> vertex4);


// Calculate the point coordinate after rotating around pivot point with angle theta
double pivotPointRotationX(double x, double y, double theta, double pivotX, double pivotY);
double pivotPointRotationY(double x, double y, double theta, double pivotX, double pivotY);


#endif
