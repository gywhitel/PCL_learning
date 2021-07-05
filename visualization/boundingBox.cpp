#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>


using pointT = pcl::PointXYZ;

// Given centroid and a vector, return the end of the vector starting from the centroid
pointT eigenvector_to_pointT(Eigen::Vector3f vector, pointT center)
{
    pointT point;
    point.x = vector(0) + center.x;
    point.y = vector(1) + center.y;
    point.z = vector(2) + center.z;
    return point;
}

// transform a point with specified rotation matrix and translation vector
pointT transform(Eigen::Vector3f translation, Eigen::Matrix3f rotation, pointT point)
{
    Eigen::Vector3f pointV;
    pointV<<point.x, point.y, point.z;
    pointV = rotation * pointV + translation;
    point.x = pointV(0);
    point.y = pointV(1);
    point.z = pointV(2);
    return point;
}

struct BB_Parameter
{
    Eigen::Vector3f BB_transform;
    Eigen::Quaternionf BB_quaternion;
    float width;
    float height;
    float depth;
};

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
    BB.depth = maxPoint.x - minPoint.x;
    BB.height = maxPoint.y - minPoint.y;
    BB.width = maxPoint.z - minPoint.z;

    return BB;

}

int main()
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (reader.read("../lamppost.pcd", *cloud) == -1)
    {
        std::cout<<"Cannot find file\n";
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr project_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pointT center;
    center.x = centroid(0);
    center.y = centroid(1);
    center.z = centroid(2);
    // origin.x = 0, origin.y = 0, origin.z = 0;

    pcl::PCA<pointT> pca;
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


    // Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    // QUESTION: What does <3,3>(0,0) mean ?
    // NOTE: I guess project_cloud is the same with cloudPointsProjected. Correct
    // projectionTransform.block<3,3>(0,0) = eigenVectors.transpose();
    // projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * centroid.head<3>());

    // get the minimum point and maximum point of the transformed cloud
    pointT minPoint, maxPoint;
    pcl::getMinMax3D(*project_cloud, minPoint, maxPoint);
    pcl::PointCloud<pointT>::Ptr point (new pcl::PointCloud<pointT>);
    point->push_back(minPoint);
    point->push_back(maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
// Quaterion of the Bounding Box determines its orientation
// TODO: Incorrect quaternion
// quaternion再转换会rotation matrix跟之前完全不一样了, 应该是因为之前的 eigenvector不是右手系
    // auto eigenvector_mod = eigenVectors;
    // eigenvector_mod.col(2) = eigenvector_mod.col(0).cross(eigenvector_mod.col(1));
    const Eigen::Quaternionf BB_quaternion(eigenVectors);
    // std::cout<<"Quaternion to rotation matrix:\n"<<BB_quaternion.toRotationMatrix()<<std::endl;
    const Eigen::Vector3f BB_transform = eigenVectors * meanDiagonal + centroid.head<3>();
    auto min = transform(BB_transform, eigenVectors, minPoint);
    auto max = transform(BB_transform, eigenVectors, maxPoint);
    point->push_back(min);
    point->push_back(max);

    pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0,0,0);
    viewer.addPointCloud(cloud, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,1, "cloud1");
    viewer.addPointCloud(project_cloud, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud2");
    viewer.addCoordinateSystem(1.0);
    viewer.addPointCloud(point, "min");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,0,0, "min");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "min");


    viewer.addArrow(ev1, center, 1,0,0,false, "x");
    viewer.addArrow(ev2, center, 0,1,0,false, "y");
    viewer.addArrow(ev3, center, 0,0,1,false, "z");

    viewer.addCube(BB_transform, BB_quaternion, maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z, "bb");
    // viewer.addCube(min.x, max.x, min.y, max.y, min.z, max.z, 0.7,0.7,0, "bb");
    viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bb");

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}

