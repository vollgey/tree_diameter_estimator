#include "diameter_estimation.hpp"

void diameter_estimation::process()
{
    std::cout << "start estimation process" << std::endl;

    pcl::visualization::CloudViewer viewer_raw("raw");
    // pcl::visualization::CloudViewer viewer_cylinder("cylinder");

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT>());

    pcl::PCDReader reader;
    reader.read<PointT> (RESOURCE_DIR"sample.pcd", *cloud);
    std::cerr << *cloud << std::endl;

    pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_filtered);
    std::cerr << *cloud_filtered << std::endl;

    estimate_normal(cloud_filtered, cloud_normals);
    double diameter_coeffs = segment_cylinder(cloud_filtered, cloud_normals, cloud_cylinder);
    std::cerr << diameter_coeffs << std::endl;

    //--------------------------------------------------
    // calcurate diameter
    //--------------------------------------------------
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloud_cylinder, minPt, maxPt);
    double diameter = maxPt.y - minPt.y;

    printf("pub points size: %5d \n", (int)cloud_cylinder->size());
    printf("diameter: %5lf [m] \n", diameter);
    printf("diameter coeffs: %5lf [m] \n", diameter_coeffs);
    printf("-------------------------- \n");
    

    viewer_raw.showCloud(cloud_cylinder);
    while (!viewer_raw.wasStopped ())
    {
    }
    // viewer_cylinder.showCloud(cloud_cylinder);
    // while (!viewer_cylinder.wasStopped ())
    // {
    // }
}

void diameter_estimation::estimate_normal(pcl::PointCloud<PointT>::Ptr input,
                        pcl::PointCloud<pcl::Normal>::Ptr output_normal)
{
    pcl::NormalEstimation<PointT, pcl::Normal> ne;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    ne.setSearchMethod(tree);
    ne.setInputCloud(input);
    ne.setKSearch(50);
    ne.compute(*output_normal);
}

double diameter_estimation::segment_cylinder(pcl::PointCloud<PointT>::Ptr input,
                      pcl::PointCloud<pcl::Normal>::Ptr input_normals,
                      pcl::PointCloud<PointT>::Ptr output)
{
    if (input->size() < 10)
        return 0;
    //instance of RANSAC segmentation processing object
    pcl::SACSegmentationFromNormals<PointT, pcl::Normal> sacseg;
    pcl::ExtractIndices<PointT> EI;
    pcl::PointIndices::Ptr inliers;
    pcl::ModelCoefficients::Ptr coeffs;
    inliers.reset(new pcl::PointIndices());
    coeffs.reset(new pcl::ModelCoefficients());
    //set RANSAC parameters
    sacseg.setOptimizeCoefficients (true);
    sacseg.setModelType (pcl::SACMODEL_CYLINDER);
    sacseg.setMethodType (pcl::SAC_RANSAC);
    sacseg.setNormalDistanceWeight (normal_distance_weight);
    sacseg.setMaxIterations (max_iterations);
    sacseg.setDistanceThreshold (distance_thres);
    sacseg.setRadiusLimits (radius_min, radius_max);
    sacseg.setInputCloud (input);
    sacseg.setInputNormals (input_normals);
    try
    {
        sacseg.segment(*inliers, *coeffs);
        EI.setInputCloud(input);
        EI.setIndices(inliers);
        EI.setNegative(false);
        EI.filter(*output);
    }
    catch (const pcl::PCLException &e)
    {
        PCL_WARN("Cylinder Model Detection Error");
        return 0; //failure
    }
    return 2*coeffs->values[6]; //success
}