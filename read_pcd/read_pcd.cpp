#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud) == -1)
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../boxes.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return(-1);
    }

    std::cout<<"Loaded "<<cloud->width * cloud->height <<" data points from"<<argv[1]<<'\n';
    pcl::visualization::CloudViewer viewer("Pointcloud");
    while(!viewer.wasStopped())
        viewer.showCloud(cloud);
    
    return 0;
}