# Normal
[show pointcloud normals](./showNormal) shows normals of a pointcloud.

This program demonstrates the usage of normal estimator and pcl visualizer that enables the display of normals.

# Registration

## [ICP](https://pcl.readthedocs.io/projects/tutorials/en/latest/iterative_closest_point.html#iterative-closest-point)

[Iterative closest point](./icp.cpp) 实现两个点云之间的匹配(pointcloudB 是由pointcloudA 平移而来), 计算出变换矩阵.

这个是直接用的PCL的API, 应该照着<14讲>里面实现一遍.

## 逐帧匹配点云

[Incrementally registration](./incrementalICP.cpp) 

# [Bounding box](http://codextechnicanum.blogspot.com/2015/04/find-minimum-oriented-bounding-box-of.html)

[bouding box](./boundingBox.cpp) 这里用的是PCA后, 将点云去中均值化, 投影到原点, 以三个特征向量为系, 求出最大最小点(模长最大, 一正一负 应该是). 然后计算出投影后点云到原点云的平移和旋转. 将bounding box变换到原点云上.

```cpp
{
    pcl::PCDReader reader;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (reader.read("../lamppost.pcd", *cloud) == -1)
    {
        std::cout<<"Cannot find file\n";
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr project_cloud (new pcl::PointCloud<pcl::PointXYZ>);
// 计算点云中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    pointT center;
    center.x = centroid(0);
    center.y = centroid(1);
    center.z = centroid(2);
 	
	pcl::PCA<pointT> pca;
    pca.setInputCloud(cloud);
    // the project here is demean, subtract the mean from the pointcloud
    pca.project(*cloud, *project_cloud);
    // Column ordered eigenvectors, representing the eigenspace cartesian basis (right-handed coordinate system).
    auto eigenVectors = pca.getEigenVectors();
    // NOTE: This line is necessary, which makes the frame right-hand. Otherwise the bounding box's orientation would be wrong.
// 这个转换是很重要的, 默认出来的eigenvector好像不是右手系, 不转换的话后面的quaternion会出错.
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    std::cout<<"PCA, Eigenvectors:\n"<<eigenVectors<<std::endl;
    auto ev1 = eigenvector_to_pointT(eigenVectors.col(0), center);
    auto ev2 = eigenvector_to_pointT(eigenVectors.col(1), center);
    auto ev3 = eigenvector_to_pointT(eigenVectors.col(2), center);
	pointT minPoint, maxPoint;
    pcl::getMinMax3D(*project_cloud, minPoint, maxPoint);
// 计算平移距离
 const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
const Eigen::Quaternionf BB_quaternion(eigenVectors);
// -------------------------- 可视化部分 ------------------
 pcl::visualization::PCLVisualizer viewer("3D viewer");
    viewer.setBackgroundColor(0,0,0);
    viewer.addPointCloud(cloud, "cloud1");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1,1,1, "cloud1");
    viewer.addPointCloud(project_cloud, "cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0,1,0, "cloud2");
viewer.addCoordinateSystem(1.0);
// 显示三个特征向量
 viewer.addArrow(ev1, center, 1,0,0,false, "x");
    viewer.addArrow(ev2, center, 0,1,0,false, "y");
    viewer.addArrow(ev3, center, 0,0,1,false, "z");

    viewer.addCube(BB_transform, BB_quaternion, maxPoint.x-minPoint.x, maxPoint.y-minPoint.y, maxPoint.z-minPoint.z, "bb");
   
 // 显示线框
viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bb");

    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}
// 将特征向量转化为原点云当中的点
pointT eigenvector_to_pointT(Eigen::Vector3f vector, pointT center)
{
    pointT point;
    point.x = vector(0) + center.x;
    point.y = vector(1) + center.y;
    point.z = vector(2) + center.z;
    return point;
}
```

`addCube`还有一种方法, 是用两个角点来添加的, 但是变换之后两个点的大小关系可能也变化了. 我这里就没有成功显示.

```cpp
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

int main()
{
    auto min = transform(BB_transform, eigenVectors, minPoint);
    auto max = transform(BB_transform, eigenVectors, maxPoint);
    
	// viewer.addCube(min.x, max.x, min.y, max.y, min.z, max.z, 0.7,0.7,0, "bb");

}
```



**Tutorial**

Eigen求解PCA

下面的代码从`cloudSegmented`开始

```cpp
// Compute principal directions

Eigen::Vector4f pcaCentroid;

pcl::compute3DCentroid(*cloudSegmented, pcaCentroid);

Eigen::Matrix3f covariance;

computeCovarianceMatrixNormalized(*cloudSegmented, pcaCentroid, covariance);

Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);

Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but the signs are different and the box doesn't get correctly oriented in some cases.
```

或者也可以使用`PCL::PCA`求解PCA

```cpp
// Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:

pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PCA<pcl::PointXYZ> pca;

pca.setInputCloud(cloudSegmented);

pca.project(*cloudSegmented, *cloudPCAprojection);

std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;

std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;

// In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
```

These eigenvectors are used to transform the point cloud to the origin point (0, 0, 0) such that the eigenvectors correspond to the axes of the space. The minimum point, maximum point, and the middle of the diagonal between these two points are calculated for the transformed cloud (also referred to as the projected cloud when using PCL's PCA interface, or reference cloud by Nicola).

```cpp
// Transform the original cloud to the origin where the principal components correspond to the axes.
Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
pcl::transformPointCloud(*cloudSegmented, *cloudPointsProjected, projectionTransform);
// Get the minimum and maximum points of the transformed cloud.
pcl::PointXYZ minPoint, maxPoint;
pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());
```

Finally, the quaternion is calculated using the eigenvectors (which determines how the final box gets rotated), and the transform to put the box in correct location is calculated. The minimum and maximum points are used to determine the box width, height, and depth.

```cpp
// Final transform
const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();

// This viewer has 4 windows, but is only showing images in one of them as written here.
pcl::visualization::PCLVisualizer *visu;
visu = new pcl::visualization::PCLVisualizer (argc, argv, "PlyViewer");
int mesh_vp_1, mesh_vp_2, mesh_vp_3, mesh_vp_4;
visu->createViewPort (0.0, 0.5, 0.5, 1.0,  mesh_vp_1);
visu->createViewPort (0.5, 0.5, 1.0, 1.0,  mesh_vp_2);
visu->createViewPort (0.0, 0, 0.5, 0.5,  mesh_vp_3);
visu->createViewPort (0.5, 0, 1.0, 0.5, mesh_vp_4);
visu->addPointCloud(cloudSegmented, ColorHandlerXYZ(cloudSegmented, 30, 144, 255), "bboxedCloud", mesh_vp_3);
visu->addCube(bboxTransform, bboxQuaternion, maxPoint.x - minPoint.x, maxPoint.y - minPoint.y, maxPoint.z - minPoint.z, "bbox", mesh_vp_3);
```

