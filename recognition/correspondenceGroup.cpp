#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include "include/addBoundingBox.h"


using PointType = pcl::PointXYZRGBA;

//Algorithm params
bool show_keypoints_ (false);
bool show_correspondences_ (false);
bool use_cloud_resolution_ (false);
bool use_hough_ (true);
float model_ss_ (0.01f);
float scene_ss_ (0.03f);
float rf_rad_ (0.015f);
float descriptor_radius (0.02f);
float cg_size_ (0.01f);
float cg_thresh_ (5.0f);

void pointXYZRGBA_to_pointXYZ(pcl::PointCloud<PointType>& cloudin, pcl::PointCloud<pcl::PointXYZ>& cloudout)
{
    cloudout.points.resize(cloudin.size());
    for (size_t i = 0; i < cloudin.size(); i++)
    {
        cloudout.points[i].x = cloudin.points[i].x;
        cloudout.points[i].y = cloudin.points[i].y;
        cloudout.points[i].z = cloudin.points[i].z;
    }
}

int main(int argc, char** argv)
{
    pcl::PointCloud<PointType>::Ptr model (new pcl::PointCloud<PointType>), model_keypoint (new pcl::PointCloud<PointType>), scene (new pcl::PointCloud<PointType>), scene_keypoint (new pcl::PointCloud<PointType>);

    pcl::PointCloud<pcl::Normal>::Ptr model_normal (new pcl::PointCloud<pcl::Normal>), scene_normal (new pcl::PointCloud<pcl::Normal>);

    pcl::PointCloud<pcl::SHOT352>::Ptr model_decriptor (new pcl::PointCloud<pcl::SHOT352>), scene_descriptor (new pcl::PointCloud<pcl::SHOT352>);

    pcl::PCDReader reader;
    reader.read("../milk.pcd", *model);
    reader.read("../milk_cartoon_all_small_clorox.pcd", *scene);

    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimation;
    normal_estimation.setKSearch(10);
    normal_estimation.setInputCloud(model);
    normal_estimation.compute(*model_normal);

    normal_estimation.setInputCloud(scene);
    normal_estimation.compute(*scene_normal);

    pcl::UniformSampling<PointType> uniform_sampling;
    uniform_sampling.setInputCloud(model);
    uniform_sampling.setRadiusSearch(model_ss_);
    uniform_sampling.filter(*model_keypoint);
    std::cout<<"Model total points: "<<model->size()<<" --> Selected keypoints: "<<model_keypoint->size()<<"\n";

    uniform_sampling.setInputCloud(scene);
    uniform_sampling.setRadiusSearch(scene_ss_);
    uniform_sampling.filter(*scene_keypoint);
    std::cout<<"Scene total points: "<<scene->size()<<" --> Selected keypoints: "<<scene_keypoint->size()<<"\n";

// compute descriptors for keypoints 
    pcl::SHOTEstimationOMP<PointType, pcl::Normal, pcl::SHOT352> descriptor_estimation;
    descriptor_estimation.setRadiusSearch(descriptor_radius);

    descriptor_estimation.setInputCloud(model_keypoint);
    descriptor_estimation.setInputNormals(model_normal);
    descriptor_estimation.setSearchSurface(model);
    descriptor_estimation.compute(*model_decriptor);

    descriptor_estimation.setInputCloud(scene_keypoint);
    descriptor_estimation.setInputNormals(scene_normal);
    descriptor_estimation.setSearchSurface(scene);
    descriptor_estimation.compute(*scene_descriptor);

//  Find correspondence with KDTree
// a vector storing all pairs
    pcl::CorrespondencesPtr model_scene_correspondence (new pcl::Correspondences);
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud(model_decriptor);

    //  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
    // 为什么用size_t 不用int ?
    for (std::size_t i = 0; i < scene_descriptor->size(); ++i)
    {
        std::vector<int> neighbor_indices (1);
        std::vector<float> neighbor_distance_indices (1);
        if (!std::isfinite(scene_descriptor->at(i).descriptor[0]))  // skip NaNs
        {
            continue;
        }
        int found_neighbor = match_search.nearestKSearch(scene_descriptor->at(i), 1, neighbor_indices, neighbor_distance_indices);
    //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
        if (found_neighbor == 1 && neighbor_distance_indices[0] <0.25f)
        {
            pcl::Correspondence correspondence(neighbor_indices[0], static_cast<int>(i), neighbor_distance_indices[0]);
            model_scene_correspondence->push_back(correspondence);
        }
    }
    std::cout<<"Found correspondence: "<<model_scene_correspondence->size()<<std::endl;
// clustering 指的是什么 ?
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rotation;
    std::vector<pcl::Correspondences> clustered_correspondences;
// cluster using Hough 3D
// reference frame for pose transformation
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_reference (new pcl::PointCloud<pcl::ReferenceFrame>), scene_reference (new pcl::PointCloud<pcl::ReferenceFrame>);

    pcl::BOARDLocalReferenceFrameEstimation<PointType, pcl::Normal, pcl::ReferenceFrame> reference_estimation;
    reference_estimation.setFindHoles(true);
    reference_estimation.setRadiusSearch(rf_rad_);

    reference_estimation.setInputCloud(model_keypoint);
    reference_estimation.setInputNormals(model_normal);
    reference_estimation.setSearchSurface(model);
    reference_estimation.compute(*model_reference);
    
    reference_estimation.setInputCloud(scene_keypoint);
    reference_estimation.setInputNormals(scene_normal);
    reference_estimation.setSearchSurface(scene);
    reference_estimation.compute(*scene_reference);
// clustering
    pcl::Hough3DGrouping<PointType, PointType, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    clusterer.setHoughBinSize(cg_size_);
    clusterer.setHoughThreshold(cg_thresh_);
    clusterer.setUseInterpolation(true);
    clusterer.setUseDistanceWeight(false);

    clusterer.setInputCloud(model_keypoint);
    clusterer.setInputRf(model_reference);
    clusterer.setSceneCloud(scene_keypoint);
    clusterer.setSceneRf(scene_reference);
    clusterer.setModelSceneCorrespondences(model_scene_correspondence);

    clusterer.recognize(rotation, clustered_correspondences);

    std::cout<<"Model instances found: "<<rotation.size()<<std::endl;

// visualization
    pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
    viewer.addPointCloud(scene, "scene");
    pcl::PointCloud<PointType>::Ptr off_scene_model (new pcl::PointCloud<PointType>), off_scene_model_keypoints (new pcl::PointCloud<PointType>);
// move the model pointcloud aside
    pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1,0,0), Eigen::Quaternionf(1, 0, 0, 0));
    pcl::transformPointCloud(*model_keypoint, *off_scene_model_keypoints, Eigen::Vector3f(-1,0,0), Eigen::Quaternionf(1, 0, 0, 0));


    pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
    viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

    for (std::size_t i = 0; i < rotation.size(); ++i)
    {   
        pcl::PointCloud<PointType>::Ptr rotated_modelrgba (new pcl::PointCloud<PointType>);
        pcl::transformPointCloud(*model, *rotated_modelrgba, rotation[i]);
        pcl::PointCloud<pointT>::Ptr rotated_model (new pcl::PointCloud<pointT>);
        pointXYZRGBA_to_pointXYZ(*rotated_modelrgba, *rotated_model);
// 原来的model经过rotation变换后就是scene当中的match
        std::stringstream ss;
        ss<<"instance"<<i;

        pcl::visualization::PointCloudColorHandlerCustom<pointT> rotated_model_color(rotated_model, 255, 0, 0);
        viewer.addPointCloud(rotated_model, rotated_model_color, ss.str());
        // show pairing lines
        
        for (std::size_t j = 0; j < clustered_correspondences[i].size(); ++j)
        {
            std::stringstream ss_line;
            ss_line<<"Correspondence_line"<<i<<"_"<<j;
            PointType model_point = off_scene_model_keypoints->at(clustered_correspondences[i][j].index_query);
            PointType scene_point = scene_keypoint->at(clustered_correspondences[i][j].index_match);
            viewer.addLine<PointType, PointType> (model_point, scene_point, 0, 255, 0, ss_line.str());
        }
        
        pointcloud_Boundingbox(rotated_model, viewer);

    } 

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}