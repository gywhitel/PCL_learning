#include <iostream>

#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/console/parse.h>
#include <pcl/common/file_io.h> // for getFilenameWithoutExtension


pcl::RangeImage::Ptr pointcloud2rangeImage(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    auto scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(cloud->sensor_origin_[0], cloud->sensor_origin_[1], cloud->sensor_origin_[2]));
    float angular_resolution = 0.5f;
    float support_size = 0.2f;
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    bool setUnseenToMaxRange = false;

    pcl::RangeImage::Ptr range_image (new pcl::RangeImage);
    range_image->createFromPointCloud(*cloud,angular_resolution, pcl::deg2rad(360.0f), pcl::deg2rad(180.0f),scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);

    return range_image;
}

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr doubleCloudVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");

  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, "cloud2");

  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

using PointType = pcl::PointXYZ;

int main(int argc, char **argv)
{
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PCDReader reader;
    reader.read("../boxes-mid.pcd", *cloud);
    /*
    if (reader.read(argv[1], *cloud) < 0)
    {
        std::cout<<"Cannot find file "<<argv[1];
        return -1;
    }
    */
// generating range image from given pointcloud
/*
    auto range_image = pointcloud2rangeImage(cloud);
    pcl::RangeImage RI = *range_image;
    pcl::RangeImageBorderExtractor RI_border_extractor;
    pcl::NarfKeypoint narf_detector(&RI_border_extractor);
    narf_detector.setRangeImage(&RI);
    narf_detector.getParameters().support_size = 0.2f;

    pcl::PointCloud<int> keypoint_indices;
    narf_detector.compute(keypoint_indices);
    std::cout<<"Found "<<keypoint_indices.size()<<" keypoints";
    pcl::PointCloud<PointType>::Ptr keypoints (new pcl::PointCloud<PointType>);
    keypoints->resize(keypoint_indices.size());
    
    for (std::size_t i = 0; i < keypoint_indices.size(); ++i)
    {
        keypoints->at(i).getVector3fMap() = range_image->at(keypoint_indices[i]).getVector3fMap();
    }
*/
    auto viewer = simpleVisual(cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}