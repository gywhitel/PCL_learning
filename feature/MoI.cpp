// Moment of Inertia  and Eccentricity based descriptor
#include <vector>
#include <thread>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

void locate(pcl::PointXYZ center)
{
    std::cout<<"The pointcloud centered at ("<<center.x<<", "<<center.y<<", "<<center.z<<")";
}



int main(int argc, char** argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::PCDReader reader;
    reader.read("../lamppost.pcd", *cloud);

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> MoIExtractor;
    MoIExtractor.setInputCloud(cloud);
    MoIExtractor.compute();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    // AABB (Axis Aligned Bounding Box), OBB (Oriented BB)
    pcl::PointXYZ min_point_AABB, max_point_AABB, min_point_OBB, max_point_OBB, position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    MoIExtractor.getMomentOfInertia(moment_of_inertia);
    MoIExtractor.getEccentricity(eccentricity);
    MoIExtractor.getAABB(min_point_AABB, max_point_AABB);
    MoIExtractor.getOBB(min_point_OBB, max_point_OBB, position_OBB,rotational_matrix_OBB);
    MoIExtractor.getEigenValues(major_value, middle_value, minor_value);
    MoIExtractor.getEigenVectors(major_vector, middle_vector, minor_vector);
    MoIExtractor.getMassCenter(mass_center);
// visualization
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("3D viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "cloud1");
// add axis aligned bounding box
/*
    viewer->addCube(min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z,max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "AABB");
*/
// add oriented bounding box
    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);
    viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "OBB");

    pcl::PointXYZ center (mass_center(0), mass_center(1), mass_center(2));
    pcl::PointXYZ x_axis (major_vector(0) + mass_center(0), major_vector(1)+ mass_center(1), major_vector(2) + mass_center(2));
    pcl::PointXYZ y_axis (middle_vector(0) + mass_center(0), middle_vector(1)+ mass_center(1), middle_vector(2) + mass_center(2));
    pcl::PointXYZ z_axis (minor_vector(0) + mass_center(0), minor_vector(1)+ mass_center(1), minor_vector(2) + mass_center(2));
    viewer->addLine(center, x_axis, 1.0f, 0.0f, 0.0f, "major eigen vector");
    viewer->addLine(center, y_axis, 0.0f, 1.0f, 0.0f, "middle eigen vector");
    viewer->addLine(center, z_axis, 0.0f, 0.0f, 1.0f, "minor eigen vector");

    while ((!viewer->wasStopped()))
    {
        using namespace std::chrono_literals;
        viewer->spinOnce(100);
        std::this_thread::sleep_for(100ms);
    }

    return(0);
    


}