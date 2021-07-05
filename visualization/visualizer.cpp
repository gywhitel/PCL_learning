#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "pcl-lib/visualization.h"

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    if (reader.read(argv[1], *cloud) < 0)
    {
        std::cout<<"Could not find file "<<argv[1];
        return -1;
    }
    // auto viewer = simpleVisual(cloud);
    auto viewer = dimensionalVisual(cloud, 0.05);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    
    return 0;
}