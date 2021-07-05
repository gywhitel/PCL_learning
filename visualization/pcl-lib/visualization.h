#ifndef VISUALIZATION_H
#define VISUALIZATION_H

# include<pcl/visualization/pcl_visualizer.h>

using pointT = pcl::PointXYZ;
using pointcloudPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

pcl::visualization::PCLVisualizer::Ptr normalVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);

pcl::visualization::PCLVisualizer::Ptr doubleViewPorts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2);

pcl::visualization::PCLVisualizer::Ptr multiVisual(std::vector<pointcloudPtr> cloud_clusters);

pcl::visualization::PCLVisualizer::Ptr dimensionalVisual(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double scale = 0.5);


#endif