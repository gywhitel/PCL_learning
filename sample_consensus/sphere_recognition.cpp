#include <iostream>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

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

using PointT = pcl::PointXYZ;

int main(int argc, char** argv)
{
    pcl::PCDReader reader;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr spherical_cloud (new pcl::PointCloud<PointT>);

    if(reader.read("/home/yinghao/Documents/realsense/realsense-development/rs2pcl/pcd/sphere-scene.pcd", *cloud) < 0)
    {
        std::cout<<"NOT found specified file.\n";
        return -1;
    }
    // pcl::PointCloud<pcl::Normal>::Ptr cloud_normal (new pcl::PointCloud<pcl::Normal>);
    // pcl::NormalEstimation<PointT, pcl::Normal> normalEstimator;
    // normalEstimator.setInputCloud(cloud);
    // pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // normalEstimator.setSearchMethod(tree);
    // normalEstimator.setRadiusSearch(0.01);
    // normalEstimator.compute(*cloud_normal);     // cannot compute normal ???

// RANSAC 
    // std::vector<int> inliers; 
    // pcl::SampleConsensusModelSphere<PointT>::Ptr model_sphere (new pcl::SampleConsensusModelSphere<PointT> (cloud));
    // pcl::RandomSampleConsensus<PointT> ransac(model_sphere);
    // ransac.setDistanceThreshold(0.01);
    // ransac.computeModel();
    // ransac.getInliers(inliers);
    // ransac.setMaxIterations(200);
// RANSAC segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::SACSegmentation<PointT> seg;
    seg.setInputCloud(cloud);
    seg.setModelType(pcl::SACMODEL_SPHERE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.05);
    seg.setOptimizeCoefficients(true);
    seg.setRadiusLimits(0.1, 0.25);
    seg.setMaxIterations(1000);
    
    pcl::PointIndices inliers;
    seg.segment(inliers, *coefficients);
    pcl::copyPointCloud(*cloud, inliers, *spherical_cloud);
    
    auto viewer = doubleCloudVisual(cloud, spherical_cloud);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}