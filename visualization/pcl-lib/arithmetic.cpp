#include "arithmetic.h"

pointT operator - (pointT p1, pointT p2)
{
    return pointT(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
}

pointT operator + (pointT p1, pointT p2)
{
    return pointT(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
}

template <typename T>
pointT operator * (pointT p1, T f)
{
    return pointT(p1.x*f, p1.y*f, p1.z*f);
}

template <typename T>
pointT operator / (pointT p1, T f)
{
    return pointT(p1.x/f, p1.y/f, p1.z/f);
}

float norm(pointT p)
{
    return sqrt( p.x * p.x + p.y * p.y + p.z * p.z);
}

// \brief add points in a vector to a pointcloud
void vectorToPointCloud(std::vector<pointT> vector, pcl::PointCloud<pointT>& cloud)
{
    auto iter = vector.begin();
    while (iter != vector.end())
    {
        cloud.push_back(*iter);
        iter++;
    }
}

std::vector<pointT> linearInterpolation(pointT start, pointT end, float step = 0.0005)
{
    std::vector<pointT> turn;
    auto direction = end - start;
    int pointNumber = norm(direction) / step;
    for (int i = 0; i < pointNumber; i++)
    {
        pointT p(start + direction / pointNumber * i);
        turn.push_back(p);
    }
    return turn;
}