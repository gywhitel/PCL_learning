#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;
using namespace pcl;
int main(int argc, char** argv)
{
    //读取STL格式模型
	vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
	reader->SetFileName(argv[1]);
	reader->Update();
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	polydata = reader->GetOutput();
	polydata->GetNumberOfPoints();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	//从ply转pcd
	pcl::io::vtkPolyDataToPointCloud(polydata, *cloud);
	//PCLVisualizer 显示原STL文件
	pcl::visualization::PCLVisualizer vis;
	vis.addModelFromPolyData(polydata, "mesh", 0);
	//保存pcd文件
	pcl::io::savePCDFileASCII(argv[2], *cloud);
	while (!vis.wasStopped())
	{
		vis.spinOnce();
	}
}
