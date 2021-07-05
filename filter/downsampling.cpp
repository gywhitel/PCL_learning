#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>

using pointT = pcl::PointXYZ;

pcl::visualization::PCLVisualizer::Ptr doubleViewPorts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addCoordinateSystem (0.2, v1);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1", v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");
//    ruler
    pointT a, b, c, d;
    a.x = 0.5, a.y = 0, a.z = 1;
    b.x = 0, b.y = 0, b.z = 1;
    c.x = 0, c.y = 0, c.z = 0.5;
    d.x = 0.5, d.y = 0, d.z = 0.5;
    viewer->addLine(a,b,"line1", v1);
    viewer->addLine(c,d, "line2", v1);
    using namespace pcl::visualization;
    {
        viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0, "line1");
        viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0, "line2");
        
    } // namespace pcl::visualization;

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->addCoordinateSystem (0.2, v2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2", v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1, 1, "cloud2");

    return viewer;
}


int main(int argc, char** argv)
{
    // difference with pcl::PointCloud ?
    // Do not have to specify point types.
    pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>());
    pcl::PointCloud<pointT>::Ptr cloud_filtered (new pcl::PointCloud<pointT>());

    pcl::PCDReader reader;
    if (reader.read(argv[1], *cloud) < 0)
    {
        std::cerr<<"Cannot find file "<<argv[1]<<'\n';
        return -1;
    }

//     std::cerr<<"Pointcloud before filtering: "<<cloud->width * cloud->height<<" data points ("<<pcl::getFieldsList(*cloud)<<")\n";
// // downsampled by voxel grid
//     pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
//     vg.setInputCloud(cloud);
//     vg.setLeafSize(0.01f, 0.01f, 0.01f);
//     vg.filter(*cloud_filtered);

// filtering outliers using statistical outlier removal filter

    pcl::StatisticalOutlierRemoval<pointT> sor;
    sor.setInputCloud(cloud);
    
    sor.setMeanK(atoi(argv[2]));
    sor.setStddevMulThresh(atof(argv[3]));
    sor.filter(*cloud_filtered);
/*
    std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl;

    pcl::PCDWriter writer;
    writer.write("downsampled.pcd", *cloud_filtered, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), false);
*/
    auto viewer = doubleViewPorts(cloud, cloud_filtered);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}