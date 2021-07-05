#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

pcl::visualization::PCLVisualizer::Ptr doubleCloudVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");

  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud2");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

using PointType = pcl::PointXYZ;

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1 (new pcl::PointCloud<pcl::PointXYZ>(5,1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);

    for (auto& point : *cloud1)
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);
        point.y = 1024 * rand() / (RAND_MAX + 1.0f); 
        point.z = 1024 * rand() / (RAND_MAX + 1.0f); 
    }

    std::cout<<"Cloud 1 has "<<cloud1->size()<<" points.\n";

    *cloud2 = *cloud1;
    for (auto& point : *cloud2)
    {
        point.x += 0.7f;
    }

    auto viewer = doubleCloudVisual(cloud1, cloud2); 
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);

    pcl::PointCloud<PointType> final;
    icp.align(final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    return 0;
}