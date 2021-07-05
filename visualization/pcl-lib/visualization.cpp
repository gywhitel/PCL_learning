#include "visualization.h"

pcl::visualization::PCLVisualizer::Ptr simpleVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  using namespace pcl::visualization;
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr normalVisual (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
{
  using namespace pcl::visualization;
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,1, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals, 1, 0.01, "normal");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 0,1,0, "normal");
  viewer->addCoordinateSystem (0.1);
  viewer->initCameraParameters ();
  return (viewer);
}


pcl::visualization::PCLVisualizer::Ptr doubleViewPorts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);	// port identifier
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addCoordinateSystem (1.0, "f1", v1);
  viewer->addPointCloud<pcl::PointXYZ> (cloud1, "cloud1", v1);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud1");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1, 1, 1, "cloud1");    

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0, 0, 0, v2);
  viewer->addCoordinateSystem (1.0, "f2", v2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2", v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1, 1, "cloud2");

    return viewer;
}

float pclcolor(float color)
{
    return color / 255;
}

pcl::visualization::PCLVisualizer::Ptr multiVisual(std::vector<pointcloudPtr> cloud_clusters)
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

// \brief show 3D dimension in pointcloud space
// \param[in] cloud: pointcloud pointer
// \param[in] scale: text scale
pcl::visualization::PCLVisualizer::Ptr dimensionalVisual(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, double scale)
{
  using namespace pcl::visualization;
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,1, "sample cloud");
  viewer->addCoordinateSystem (0.10);
  viewer->initCameraParameters ();
  std::stringstream ss;
  int id = 1;
    
  // 3 axis
  viewer->addArrow(pointT(1,0,0), pointT(-1, 0, 0), 1,1,0, false, "x");
  viewer->addArrow(pointT(0,1,0), pointT(0, -1, 0), 1,1,0, false, "y");
  viewer->addArrow(pointT(0,0,1), pointT(0, 0, 0), 1,1,0, false, "z");
  // viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,"z");
  for (float i = 0.5; i < 1.5; i += 0.5)
  {
    // x axis
    ss<<"line"<<id;
    viewer->addLine(pointT(0,0,i), pointT(1, 0, i), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    ss.str("");
    ss<<"x="<<i;
    viewer->addText3D(ss.str(), pointT(i, 0, 0), scale, 1,1,0, ss.str());
    id++;
    ss.str("");

  // y axis
    ss<<"line"<<id;
    viewer->addLine(pointT(0,0,i), pointT(0, 1, i), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    ss.str("");
    ss<<"y="<<i;
    viewer->addText3D(ss.str(), pointT(0, i, 0), scale, 1,1,0, ss.str());
    id++;
    ss.str("");
  
  // z axis
    ss<<"z="<<i;
    viewer->addText3D(ss.str(), pointT(0, 0, i), scale, 1,1,0, ss.str());
    id++;
    ss.str("");
    // grid
    ss<<"line"<<id;
    viewer->addLine(pointT(i,0,0), pointT(i, 0, 1), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");

    ss<<"line"<<id;
    viewer->addLine(pointT(-i,0,0), pointT(-i, 0, 1), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");

    ss<<"line"<<id;
    viewer->addLine(pointT(0,i,0), pointT(0, i, 1), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");

    ss<<"line"<<id;
    viewer->addLine(pointT(0,-i,0), pointT(0,-i, 1), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");

    ss<<"line"<<id;
    viewer->addLine(pointT(0,0,i), pointT(0, -1, i), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");
    
    ss<<"line"<<id;
    viewer->addLine(pointT(0,0,i), pointT(-1, 0, i), ss.str());
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,ss.str());
    id++;
    ss.str("");

  }
  return (viewer);
}
