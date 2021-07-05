#include <iostream>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>    // moving least square

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

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr passed_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);



    if (pcl::io::loadPCDFile<pcl::PointXYZ>("../boxes-mid.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file\n");
        return(-1);
    }

    std::cout<<"Loaded "<<cloud->width * cloud->height <<" data points\n";

    // downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*filtered_cloud);
    std::cout<<"The downsampled pointcloud has "<<filtered_cloud->width * filtered_cloud->height<<" points\n";
    
    // pass through filter， remove far points
/*
    pcl::PassThrough<pcl::PointXYZ> pass1(true);
    pass1.setInputCloud(filtered_cloud);
    pass1.setFilterFieldName("x");
    pass1.setFilterLimits(-0.5,0.5);
    pass1.filter(*passed_cloud);
    // pcl::PassThrough<pcl::PointXYZ> pass2;
    // pass2.setInputCloud(passed_cloud);
    pass1.setInputCloud(passed_cloud);
    pass1.setFilterFieldName("z");
    pass1.setFilterLimits(0.5,1.2);
    pass1.filter(*passed_cloud2);
    passed_cloud->~PointCloud();
*/
    // 之前的指针用不用销毁 ？
    // delete &filtered_cloud;
    /*
    // create a kd-tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    // store normal calculated by mls
    pcl::PointCloud<pcl::PointNormal> mls_points;
    // initiate mls object
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    mls.setInputCloud(cloud);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.03);

    mls.process(mls_points);
    */
    // pcl::visualization::CloudViewer viewer("Pointcloud before filtering");
    // viewer.showCloud(cloud);

    // viewer.runOnVisualizationThreadOnce(viewerOnce);
    
    // create a filtering object
    // remove outliers
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
    outlier_filter.setInputCloud(cloud);
    outlier_filter.setMeanK(50);
    outlier_filter.setStddevMulThresh(1.0);
    outlier_filter.filter(*filtered_cloud);

    auto viewer = simpleVisual(filtered_cloud);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}