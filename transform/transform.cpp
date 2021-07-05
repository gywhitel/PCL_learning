#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/visualization/pcl_visualizer.h>

auto transform(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float theta, float x, float y, float z)
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // Rotation about z axis
    transform(0,0) = std::cos (theta);
    transform(0,1) = -sin(theta);
    transform(1,0) = sin (theta);
    transform(1,1) = std::cos (theta);

    //translation
    transform(0,3) = x;
    transform(1,3) = y;
    transform(2,3) = z;

    std::cout<<"The homogenous transformation matrix is \n"<<transform<<"\n";
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return transformed_cloud;
}

pcl::visualization::PCLVisualizer::Ptr doubleCloudVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 255, 255, 255, "cloud1");

  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 255, 0, "cloud2");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;
    if (reader.read(argv[1], *cloud) < 0)
    {
        std::cout<<"Cannot find file "<<argv[1]<<"\n";
        return -1;
    }
    pcl::PointXYZ centroid;
    pcl::computeCentroid(*cloud, centroid);
    
    
    auto transformed_cloud = transform(cloud, 0, -centroid.x, 0, 0);
    
    auto viewer = doubleCloudVisual(cloud, transformed_cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    } 
    return 0;
}