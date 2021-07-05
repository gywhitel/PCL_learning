#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>


pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

float pclcolor(float color)
{
    return color / 255;
}

using pointcloudPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;
// using xyzCloud = pcl::PointCloud<pcl::PointXYZ>;

auto multiVisual(std::vector<pointcloudPtr> cloud_clusters)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    std::stringstream ss;
    int i = 1;
    for (auto cloud = cloud_clusters.begin(); cloud != cloud_clusters.end(); ++cloud)
    {
        ss<<"Pointcloud"<<i;
        std::cout<<ss.str()<<" has : "<<(*cloud)->size()<<"points.\n";
        viewer->addPointCloud<pcl::PointXYZ> ((*cloud), ss.str());
        viewer->addCoordinateSystem (1.0);
        float r = pclcolor(250 - 30*i), g = pclcolor(160+20*i), b = pclcolor(50*i);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, r,g,b, ss.str());
        viewer->initCameraParameters ();
        ++i;
        ss.str("");
    }

    return viewer;
}



int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_seg (new pcl::PointCloud<pcl::PointXYZ>);


    pcl::PCDReader reader;
    reader.read("../boxes-mid.pcd", *cloud);
    pcl::PCDWriter writer;

// downsampling
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.01f, 0.01f, 0.01f);
    voxel.filter(*filtered_cloud);
    std::cout<<"The downsampled pointcloud has "<<filtered_cloud->width * filtered_cloud->height<<" points\n";

// segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.015);

// filter object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0;
    int nr_points = (int) filtered_cloud->size(); // c-style type conversion
    std::vector<pointcloudPtr> cloud_clusters;
    while (filtered_cloud->size() > 0.3 * nr_points)
    {
        seg.setInputCloud(filtered_cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            std::cout<<"Cannot estimate a planar model from the given dataset.\n";
            break;
        }

        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
// get the points on the planar surface
        extract.filter(*cloud_plane);
        // pcl::PointCloud<pcl::PointXYZ> cloud_temp = *cloud_plane;
        pointcloudPtr ptr (new pcl::PointCloud<pcl::PointXYZ> (*cloud_plane));
        cloud_clusters.push_back(ptr);
        std::cout<<"pointcloud representign the planar component: "<<cloud_plane->size() <<" data points.\n";
    /*
        std::stringstream ss;
        ss<<"Pointcloud_"<<i<<".pcd";
        std::cout<<"Generate "<<ss.str()<<"\n";
        writer.write<pcl::PointXYZ>(ss.str(), *cloud_plane, false);
    */
//  filter
        extract.setNegative(true);
        extract.filter(*cloud_f);
// update the pointcloud, remove the fitted points
        *filtered_cloud = *cloud_f;
        i++;
    }

    auto viewer = multiVisual(cloud_clusters);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}