#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#define R 0.01

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr normalsVisual (
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color(cloud, 250,250,250);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, color, "base cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "base cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 50, 0.1, "normals");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1, 0, "normals");
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

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PCDReader reader;
    reader.read("../boxes-mid.pcd", *cloud);
    std::cout<<"read "<<cloud->width * cloud->height<<" data points\n";

    bool downsampling = false; 
// downsampling using voxel grid to lower computational cost
    if (downsampling)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(0.01f, 0.01f, 0.01f);
        voxel.filter(*cloud_down);
        std::cout<<"The pointcloud after downsampling has "<<cloud_down->width * cloud_down->height<<" data points\n";
    }

// create normal estimation class
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud);
// set search method -- kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimator.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
// TODO: ????????????????????????????????????
    normalEstimator.setRadiusSearch(R);
    // normalEstimator.getSearchMethod()
    normalEstimator.compute(*cloud_normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_with_normals);

    // auto viewer = normalsVisual(cloud, cloud_normals);
    auto viewer = normalsVisual(cloud_with_normals);

    while (!viewer->wasStopped())
    {
        // call spinOnce to update the screen
        viewer->spinOnce (100);
    }

    return 0;
}