//
// Created by 丁昊 on 9/22/19.
//

#ifndef TEMPLATE_UTIL_H
#define TEMPLATE_UTIL_H

#include <iostream>
#include <utility>
#include <vector>

using namespace std;

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

#endif //TEMPLATE_UTIL_H
