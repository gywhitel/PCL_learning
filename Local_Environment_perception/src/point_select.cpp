#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <fstream>
// #include <thread>

#include <sys/socket.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>


#define STEP 0.0001     // m
#define START -0.20     // m
#define END 0.18        // m
#define EPSILON 0.005   // m



using pointT = pcl::PointXYZ;

pcl::visualization::PCLVisualizer::Ptr doubleCloudVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");

  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVisual(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_normal)
{
  using namespace pcl::visualization;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointNormal> (cloud_normal, "points");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,1, "points");

  viewer->addPointCloudNormals<pcl::PointNormal, pcl::PointNormal>(cloud_normal, cloud_normal, 50, 0.02, "normals");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 0,1,0, "normals");
  viewer->addCoordinateSystem(0.1);
  viewer->initCameraParameters();
  return (viewer);
}


struct point_2D
{
    float y;
    float z;
};

// \brief average a point neighborhood
// \param[in] neighbor: a vector of 2D points in a neighborhood
point_2D average(std::vector<pointT> neighbor)
{
    float sumY = 0, sumZ = 0;
    for (auto point: neighbor)
    {
        sumY += point.y;
        sumZ += point.z;
    }
    auto averageY = sumY / neighbor.size();
    auto averageZ = sumZ / neighbor.size();

    point_2D average_point;
    average_point.y = averageY;
    average_point.z = averageZ;

    return average_point;
}

// \brief 提取输入点云的一个截面, 将点云平均为 (end - start)/step 个点, 每个点为周围step邻域的均值
// \param[in] cloud: pointcloud pointer
// \param[in] start: the start of the path
// \param[in] end: the end of the path
// \param[in] step: neighborhood radius
std::vector<point_2D> neighbor_average(pcl::PointCloud<pointT>::ConstPtr cloud, const float start, const float end, const float step)
{   
    std::vector<point_2D> sorted_point;
    for (float i = start; i <= end; i += step)
    {   
        std::vector<pointT> neighbor;
        for (auto &p : *cloud)
        {
            // select neighbor points

            if ( abs(p.y - i) <= step)
            {
                neighbor.push_back(p);
            }
        }
        if ( neighbor.size() == 0 )
        {
            continue;
        }
        auto averagePoint = average(neighbor);
        sorted_point.push_back(averagePoint);
    }
    return sorted_point;
}


int main(int argc, char** argv)
{
    pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>);
    pcl::PCDReader reader;
    if (reader.read(argv[1], *cloud) == -1)
    {
        std::cout<<"Cannot find file "<<argv[1]<<"\n";
        return -1;
    }

    pcl::PointCloud<pointT>::Ptr section (new pcl::PointCloud<pointT>);

    pcl::PassThrough<pointT> passThru;
    passThru.setInputCloud(cloud);
    passThru.setFilterFieldName("z");
    passThru.setFilterLimits(0.0, 0.7);
    passThru.filter(*cloud);

    for (auto &point: *cloud)
    {
        if (abs(point.x - 0) <= EPSILON)
        {
            section->push_back(point);
        }
    }
    std::cout<<"Select "<<section->size()<<" points from original pointcloud\n";

// -0.20 ~ 0.18
    std::vector<pointT> sectionLine;
    const float step = 0.0001;  // 0.1mm
    // const int interval = (0.18-(-0.20)) / step;
    auto sorted_points = neighbor_average(section, START, END, STEP);
    std::cout<<sorted_points.size()<<" after averaging\n";
/*
    ofstream file;
    file.open("data.txt");
    for (auto point : sorted_points)
    {
        file<<point.y<<" "<<point.z<<"\n";
    }
    file.close();
*/ 
/*

    auto viewer = doubleCloudVisual(cloud, section);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
*/
    return 0;
}