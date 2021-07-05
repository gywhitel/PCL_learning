#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

using pointT = pcl::PointXYZ;

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
    pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>());
    pcl::PCDReader reader;
    // reader.read(argv[1], *cloud);
    reader.read("../pcd/sin-surface.pcd", *cloud);

    Eigen::Vector4f centroid;
    pointT center;
    // QUESTION: 这里用vector竟然不行了 ??? 之前都是可以的
    pcl::computeCentroid(*cloud, center);
    Eigen::Matrix4f offset = Eigen::Matrix4f::Identity();
    offset(0,3) = -center.x;
    offset(1,3) = -center.y;
    offset(2,3) = -center.z;
    pcl::PointCloud<pointT>::Ptr moved_cloud (new pcl::PointCloud<pointT>());
    pcl::transformPointCloud(*cloud, *moved_cloud, offset);
    auto viewer = simpleVisual(moved_cloud);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }
    return 0;
}