#ifndef DIAMETER_ESTIMATION_H
#define DIAMETER_ESTIMATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>
#include <pcl/exceptions.h>

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>
#include <math.h>
#include <iostream>

typedef pcl::PointXYZRGB PointT;

class diameter_estimation
{
private:

public:
    diameter_estimation(){};
    virtual ~diameter_estimation(){};
    
    void process();
    void remove_noise(pcl::PointCloud<PointT>::Ptr input,
                        pcl::PointCloud<PointT>::Ptr output);
    void downsample(pcl::PointCloud<PointT>::Ptr input,
                        pcl::PointCloud<PointT>::Ptr output,
                        double leaf_size);
    void passthrough(pcl::PointCloud<PointT>::Ptr input,
                          pcl::PointCloud<PointT>::Ptr output
                          );
    void estimate_normal(pcl::PointCloud<PointT>::Ptr input,
                          pcl::PointCloud<pcl::Normal>::Ptr output_normal);             
    double segment_cylinder(pcl::PointCloud<PointT>::Ptr input,
                        pcl::PointCloud<pcl::Normal>::Ptr input_normals,
                        pcl::PointCloud<PointT>::Ptr output);


    //parameters(downsample)
    double leaf_size = 0.005;

    //parameters(passthrough)
    double pt_min = 0.0;
    double pt_max = 0.4;

    //parameters(cylinder_segmentation)
    double normal_distance_weight = 0.01;
    double max_iterations = 10000; 
    double distance_thres = 0.005; 
    double radius_min = 0;
    double radius_max = 0.2;
};                    
#endif