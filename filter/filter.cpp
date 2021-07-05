#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("test_pcd.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return(-1);
    }

    std::cout<<"Loaded "<<cloud->width * cloud->height <<" data points from test_pcd.pcd.\n";

    // cloud->width  = 100;
    // cloud->height = 1;
    // cloud->points.resize(cloud->width * cloud->height);

    // for (auto& point: *cloud)
    // {
    //     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     point.z #pragma endregion= 1024 * rand () / (RAND_MAX + 1.0f);
    // }

    // create a filter object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 1.0);
    pass.filter(*filtered_cloud);

    pcl::io::savePCDFileASCII("filtered.pcd", *filtered_cloud);
    std::cerr<<"Saved "<<filtered_cloud->size()<<" data points to filtered.pcd.\n";
    
    return 0;
}