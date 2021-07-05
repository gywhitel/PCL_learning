#include <iostream>
#include <thread>

#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>

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

int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), final (new pcl::PointCloud<pcl::PointXYZ>);
    // populate our PointCloud with points
    cloud->width    = 500;
    cloud->height   = 1;
    cloud->is_dense = false;
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i = 0; i < cloud->size (); ++i)
    {
        (*cloud)[i].x = 1024 * rand () / (RAND_MAX + 1.0);
        (*cloud)[i].y = 1024 * rand () / (RAND_MAX + 1.0);
        if (i % 5 == 0)
            (*cloud)[i].z = 1024 * rand () / (RAND_MAX + 1.0);
        else if(i % 2 == 0)
            (*cloud)[i].z =  sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                        - ((*cloud)[i].y * (*cloud)[i].y));
        else
            (*cloud)[i].z =  - sqrt( 1 - ((*cloud)[i].x * (*cloud)[i].x)
                                            - ((*cloud)[i].y * (*cloud)[i].y));
    }

    std::vector<int> inliers;

    // create RANSAC object and compute appropriated model
    // spherical model fitting
    pcl::SampleConsensusModelSphere<pcl::PointXYZ>::Ptr model_sphere (new pcl::SampleConsensusModelSphere<pcl::PointXYZ> (cloud));

    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_sphere);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    ransac.getInliers(inliers);
    pcl::copyPointCloud(*cloud, inliers, *final);

    auto viewer = doubleCloudVisual(cloud, final);
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return 0;
}