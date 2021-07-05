#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>


using PointT = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointT>;
using PointNormal = pcl::PointNormal;
using PointCloudNormal = pcl::PointCloud<PointNormal>;

struct PCD
{
    PointCloud::Ptr cloud;
    std::string file_name;    
};

void loadData(int argc, char** argv, std::vector<PCD>& data)
{
     for (int i = 1; i < argc; i++)
    {
        PCD scene;
        scene.file_name = argv[i];
        pcl::io::loadPCDFile(argv[i], *scene.cloud);
        // remove NaN points
        std::vector<int> indices;
        pcl::removeNaNNormalsFromPointCloud(*scene.cloud, *scene.cloud, indices);
        data.push_back(scene);
    }
}

void pairAlign(const PointCloud::Ptr sourceCloud, const PointCloud::Ptr targetCloud, PointCloud::Ptr outputCloud, Eigen::Matrix4f &final_transform, bool downsample = true)
{
    // Downsampling or NOTs
    PointCloud::Ptr source (new PointCloud), target (new PointCloud);
    pcl::VoxelGrid<PointT> voxel;
    if (downsample)
    {
        voxel.setLeafSize(0.01, 0.01, 0.01);
        voxel.setInputCloud(sourceCloud);
        voxel.filter(*source);

        voxel.setInputCloud(targetCloud);
        voxel.filter(*target);
    }
    else
    {
        source = sourceCloud;
        target = targetCloud;
    }

    PointCloudNormal::Ptr source_cloud_normal (new PointCloudNormal);
    PointCloudNormal::Ptr target_cloud_normal (new PointCloudNormal);

    pcl::NormalEstimation<PointT, PointNormal> normal_estimator;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setKSearch(30);

    normal_estimator.setInputCloud(source);
    normal_estimator.compute(*source_cloud_normal);

    normal_estimator.setInputCloud(target);
    normal_estimator.compute(*target_cloud_normal);

    // Align
    pcl::IterativeClosestPointNonLinear<PointNormal, PointNormal> regstr;
    regstr.setTransformationEpsilon(1e-6);
    // set the maximum distance between two correspondences ???
    // TODO: figure out how the pcl ICP works. 
    regstr.setMaxCorrespondenceDistance(0.1);
}

int main(int argc, char** argv)
{
    std::vector<PCD> data;
    loadData(argc, argv, data);

    auto viewer = pcl::visualization::PCLVisualizer("Pairwise Incremental Registration");

    PointCloud::Ptr result(new PointCloud), source, target;
    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    for (size_t i = 1; i < data.size(); ++i);
    {
        source = data[i-1].cloud;
        target = data[i].cloud;

        // visualization

    }
}