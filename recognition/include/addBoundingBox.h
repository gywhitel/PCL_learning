#ifndef BB_H
#define BB_H

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using pointT = pcl::PointXYZ;
// using pointT = pcl::PointXYZRGBA;


// a struct contains parameters to render a bounding box
struct BB_Parameter
{
    Eigen::Vector3f BB_transform;
    Eigen::Quaternionf BB_quaternion;
    float width;
    float height;
    float depth;
    pointT center, eigenvector_point1, eigenvector_point2, eigenvector_point3;

};

// Given centroid and a vector, return the end of the vector starting from the centroid
// \param vector a translation vector
// \param center a pointT type point, the centroid of the pointcloud
pointT eigenvector_to_pointT(Eigen::Vector3f vector, pointT center);

// \brief add a bounding box on given input cloud
// \param[in] cloud a pointcloud pointer
// \param[out] BB a BB_Parameter struct
BB_Parameter pointcloud_Boundingbox(pcl::PointCloud<pointT>::ConstPtr cloud);

void pointcloud_Boundingbox(pcl::PointCloud<pointT>::ConstPtr cloud, pcl::visualization::PCLVisualizer& viewer);

// \brief Add a bounding box and orientation in a given PCL_Visualizer
// \param[in] viewer the viewer to which the bounding box is added
// \param[in] BB a BB_Parameter struct, containing all the parameters needed to render a bounding box
void addBoundingBox(pcl::visualization::PCLVisualizer& viewer, BB_Parameter BB);

#endif