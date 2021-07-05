#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>


using PointType = pcl::PointXYZ;
int main(int argc, char** argv)
{
    pcl::PointCloud<PointType> cloud;
    pcl::PCDReader reader;
    if (reader.read("../boxes-mid.pcd", cloud) < 0)
    {
        std::cout<<"Cannot find file "<<argv[1]<<'\n';
        return -1;
    }

    float angularResolution = (float)(1.0f * (M_PI/180.0f));
    float maxAngleWidth = (float)(360.0f * (M_PI/180.0f));
    float maxAngleHeight = (float)(360.0f * (M_PI/180.0f));

    auto sensorPose = Eigen::Affine3f(Eigen::Translation3f(cloud.sensor_origin_[0], cloud.sensor_origin_[1], cloud.sensor_origin_[2]) * Eigen::Affine3f(cloud.sensor_orientation_));
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;

    float noiseLevel = 0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(cloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

    pcl::visualization::RangeImageVisualizer range_image_viewer("Range Image");
    range_image_viewer.showRangeImage(rangeImage);
}