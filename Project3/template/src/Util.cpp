//
// Created by Hao Ding on 9/22/19.
//

#include "Util.h"

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

