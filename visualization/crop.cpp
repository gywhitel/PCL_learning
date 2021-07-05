#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <thread>
#include <iostream>
#include <unistd.h>

using pointT = pcl::PointXYZ;

void printUsage (const char* progName)
{
  std::cout << "\n\nUsage: "<<progName<<" [options] [parameter]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "x           crop in x direction\n"
            << "y           crop in y direction\n"
            << "z           crop in z direction\n"
            << "\n\n"
            <<"parameters: start end /m\n";
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

pcl::visualization::PCLVisualizer::Ptr doubleViewPorts(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud2)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addCoordinateSystem (1.0, v1);
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
  viewer->addCoordinateSystem (1.0, v2);
  viewer->addPointCloud<pcl::PointXYZ> (cloud2, "cloud2", v2);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud2");
   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 1, 1, "cloud2");

    return viewer;
}

void savePointCloud(pcl::PointCloud<pointT>::ConstPtr cloud)
{
    char flag=' ';
    while (flag != 's')
    {
        flag = getchar();
    } 
    std::cout<<'"'<<flag<<"\" was pressed.\n";
    std::cout<<"The cloud has "<<cloud->size()<<" points\n";
    std::cout<<"Ready to save the cropped pointcloud. Please name the pointcloud...\n";
    std::string filename;
    // QUESTION: Why do I have to use getline twice to make the input work ?
    getline(std::cin, filename);
    getline(std::cin, filename);
    // std::stringstream ss;
    // ss<<"../"
    pcl::PCDWriter writer;
    writer.write(filename, *cloud);
    std::cout<<filename<<" was saved.\n";
}

int main(int argc, char** argv)
{
    
    if ( argc == 2 || 5 )
    {
        if (argc == 2)
        {
            pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>);
            pcl::PointCloud<pointT>::Ptr cropCloud (new pcl::PointCloud<pointT>);

            pcl::PCDReader reader;
            assert(reader.read(argv[1], *cloud) != -1);
            auto viewer = simpleVisual(cloud);
            while (!viewer->wasStopped())
            {
                viewer->spinOnce();
            }
            return 0;
        }
        else
        {
            if ( argv[2] == "x" || "y" || "z" )
            {
                pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>);
                pcl::PointCloud<pointT>::Ptr cropCloud (new pcl::PointCloud<pointT>);
                pcl::PCDReader reader;
                assert(reader.read(argv[1], *cloud) != -1);

                pcl::PassThrough<pointT> passThru;
                passThru.setInputCloud(cloud);
                passThru.setFilterFieldName(argv[2]);
                float min = atof(argv[3]); 
                float max = atof(argv[4]);
                passThru.setFilterLimits(min, max);
                passThru.filter(*cropCloud);


                auto viewer = doubleViewPorts(cloud, cropCloud);
                std::cout<<"Press 's' to save the cropped pointcloud\n";

                std::thread test(savePointCloud, cropCloud);
                test.detach();
                while (!viewer->wasStopped())
                {
                    viewer->spinOnce();
                }
                
                return 0;
            }
            else
            {
                printUsage(argv[0]);
                return 0;
            }
        }

    }
    else
    {
        printUsage(argv[0]);
        return 0;
    }
}