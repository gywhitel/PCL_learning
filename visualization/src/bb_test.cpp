#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>

using pointT = pcl::PointXYZ;

// a struct contains parameters to render a bounding box
struct BB_Parameter
{
    Eigen::Vector3f BB_transform;
    Eigen::Quaternionf BB_quaternion;
    float width;
    float height;
    float depth;
    pointT center, eigenvector_point1, eigenvector_point2, eigenvector_point3;

};


// Given centroid and a vector, return the end of the vector starting from the centroid
// \param vector a translation vector
// \param center a pointT type point, the centroid of the pointcloud
pointT eigenvector_to_pointT(Eigen::Vector3f vector, pointT center)
{
    pointT point;
    point.x = vector(0) + center.x;
    point.y = vector(1) + center.y;
    point.z = vector(2) + center.z;
    return point;
}

// \brief add a bounding box on given input cloud
// \param[in] cloud a pointcloud pointer
// \param[out] BB a BB_Parameter struct
BB_Parameter pointcloud_Boundingbox(pcl::PointCloud<pointT>::ConstPtr cloud)
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pointT center;
    center.x = centroid(0);
    center.y = centroid(1);
    center.z = centroid(2);
    // Principal Component Analysis
    pcl::PCA<pointT> pca;
    pcl::PointCloud<pcl::PointXYZ>::Ptr project_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pca.setInputCloud(cloud);
    // the project here is demean, subtract the mean from the pointcloud
    pca.project(*cloud, *project_cloud);
    // Column ordered eigenvectors, representing the eigenspace cartesian basis (right-handed coordinate system).
    auto eigenVectors = pca.getEigenVectors();
    // NOTE: This line is necessary, which makes the frame right-hand. Otherwise the bounding box's orientation would be wrong.
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    std::cout<<"PCA, Eigenvectors:\n"<<eigenVectors<<std::endl;
    auto ev1 = eigenvector_to_pointT(eigenVectors.col(0), center);
    auto ev2 = eigenvector_to_pointT(eigenVectors.col(1), center);
    auto ev3 = eigenvector_to_pointT(eigenVectors.col(2), center);
    pointT minPoint, maxPoint;
    pcl::getMinMax3D(*project_cloud, minPoint, maxPoint);
    // Get translation vector and rotation matrix
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
    const Eigen::Vector3f BB_transform = eigenVectors * meanDiagonal + centroid.head<3>();
    const Eigen::Quaternionf BB_quaternion(eigenVectors);
    BB_Parameter BB;
    BB.BB_quaternion = BB_quaternion;
    BB.BB_transform = BB_transform;
    BB.width = maxPoint.x - minPoint.x;
    BB.height = maxPoint.y - minPoint.y;
    BB.depth = maxPoint.z - minPoint.z;
    BB.center = center;
    BB.eigenvector_point1 = ev1;
    BB.eigenvector_point2 = ev2;
    BB.eigenvector_point3 = ev3;
    return BB;

}

// \brief Add a bounding box and orientation in a given PCL_Visualizer
// \param[in] viewer the viewer to which the bounding box is added
// \param[in] BB a BB_Parameter struct, containing all the parameters needed to render a bounding box
void addBoundingBox(pcl::visualization::PCLVisualizer& viewer, BB_Parameter BB)
{   
    using namespace pcl::visualization;
    viewer.addCube(BB.BB_transform, BB.BB_quaternion,BB.width, BB.height, BB.depth, "bb");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bb");
    viewer.setShapeRenderingProperties(PCL_VISUALIZER_COLOR, 1,1,0,"bb");
    viewer.addArrow(BB.eigenvector_point1, BB.center, 1,0,0,false, "x");
    viewer.addArrow(BB.eigenvector_point2, BB.center, 0,1,0,false, "y");
    viewer.addArrow(BB.eigenvector_point3, BB.center, 0,0,1,false, "z");
}

int main()
{
    pcl::PCDReader reader;
    pcl::PointCloud<pointT>::Ptr cloud (new pcl::PointCloud<pointT>);
    if (reader.read("../lamppost.pcd", *cloud) == -1)
    {
        std::cout<<"Cannot find file\n";
        return -1;
    }
    auto BB = pointcloud_Boundingbox(cloud);

    auto viewer = pcl::visualization::PCLVisualizer("Bounding box");
    viewer.setBackgroundColor(0,0,0);
    viewer.addPointCloud(cloud, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,1, "cloud1");

    addBoundingBox(viewer, BB);

    

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}