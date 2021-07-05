#ifndef ARITHMETIC_H
#define ARITHMETIC_H

#include <pcl/common/common_headers.h>
#include <vector>

using pointT = pcl::PointXYZ;

pointT operator - (pointT p1, pointT p2);

pointT operator + (pointT p1, pointT p2);

template <typename T> pointT operator * (pointT p1, T f);

template <typename T> pointT operator / (pointT p1, T f);

float norm(pointT p);

// \brief add points in a vector to a pointcloud
void vectorToPointCloud(std::vector<pointT> vector, pcl::PointCloud<pointT>& cloud);

// \brief Linearly interpolate points between start and end
std::vector<pointT> linearInterpolation(pointT start, pointT end, float step = 0.0005);

#endif