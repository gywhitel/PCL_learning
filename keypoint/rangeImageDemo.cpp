#include <pcl/range_image/range_image.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

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


int main (int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ> &cloud = *cloudPtr;
/*
  pcl::PCDReader reader;
  reader.read("../boxes-mid.pcd", cloud);
  
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(cloudPtr);
  voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  voxel.filter(cloud);
*/
//   auto viewer = simpleVisual(cloud)
  
  // Generate the data
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      cloud.points.push_back(point);
    }
  }
  cloud.width = cloud.size();
  cloud.height = 1;
  
 
  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
//   Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    auto sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud.sensor_origin_[0], cloud.sensor_origin_[1], cloud.sensor_origin_[2]) * Eigen::Affine3f(cloud.sensor_orientation_));

  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  float noiseLevel=0.00;
  float minRange = 0.0f;
  int borderSize = 1;
  
  pcl::RangeImage::Ptr range_image_ptr (new pcl::RangeImage);
  pcl::RangeImage &rangeImage = *range_image_ptr;

  rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";

  pcl::visualization::PCLVisualizer viewer("3D viewer");
  viewer.setBackgroundColor(0,0,0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(range_image_ptr, 1,1,1);
  viewer.addPointCloud(range_image_ptr, range_image_color_handler, "range image");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "range image");
  viewer.initCameraParameters();

  pcl::visualization::RangeImageVisualizer range_image_viewer("Range Image");
  range_image_viewer.showRangeImage(rangeImage);

  while (!viewer.wasStopped())
  {
      range_image_viewer.spinOnce(100);
      viewer.spinOnce(100);
  }

 return 0;
}