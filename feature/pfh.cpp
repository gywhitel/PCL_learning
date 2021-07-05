#include <iostream>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/fpfh.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud  (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_down (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal> ());   

    pcl::PCDReader reader;
    reader.read("../boxes-mid.pcd", *cloud);
    std::cout<<"read "<<cloud->width * cloud->height<<" data points\n";

    // downsampling using voxel grid
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*cloud_down);
    std::cout<<"The pointcloud after downsampling has "<<cloud_down->width * cloud_down->height<<" data points\n";

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setInputCloud(cloud_down);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    normalEstimator.setSearchMethod(tree);
// too computation-costly without downsampling

    normalEstimator.setRadiusSearch(0.05);
    normalEstimator.compute(*normal);

// Fast PFH estimation class
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
    
    fpfh.setInputCloud(cloud_down);
    fpfh.setInputNormals(normal);
    fpfh.setRadiusSearch(0.05);
    fpfh.compute(*fpfhs);
    
    pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter();
    plotter->addFeatureHistogram(*fpfhs,11);
    plotter->plot();
    return 0;
}