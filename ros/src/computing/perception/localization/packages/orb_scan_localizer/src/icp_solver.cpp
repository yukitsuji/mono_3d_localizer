#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/transformation_estimation_lm.h>

#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

int main (int argc, char** argv){
    setVerbosityLevel(L_DEBUG);

    //点群を宣言する。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    //点群の移動量を設定
    double x_offset=0.7f;
    double y_offset=0.5f;

    //点群cloud_inにデータを入力
    cloud_in->width    = 10;
    cloud_in->height   = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize (cloud_in->width * cloud_in->height);

    for (size_t i = 0; i < cloud_in->points.size (); ++i)   {
        cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << cloud_in->points.size () << " data points to input:"<< std::endl;
    for (size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
    cloud_in->points[i].x << " " <<
    cloud_in->points[i].y << " " <<
    cloud_in->points[i].z << std::endl;
    *cloud_out = *cloud_in;
    std::cout << "size:" << cloud_out->points.size() << std::endl;

    //点群を移動させcloud_outを作る。
    for (size_t i = 0; i < cloud_in->points.size (); ++i){
        cloud_out->points[i].x = cloud_in->points[i].x + x_offset + 102 * rand () / (RAND_MAX + 1.0f);
        cloud_out->points[i].y = cloud_in->points[i].y + y_offset + 102 * rand () / (RAND_MAX + 1.0f);

    }
    std::cout << "Transformed " << cloud_in->points.size () << " data points:"  << std::endl;

    for (size_t i = 0; i < cloud_out->points.size (); ++i)
    std::cout << "    " <<
    cloud_out->points[i].x << " " <<
    cloud_out->points[i].y << " " <<
    cloud_out->points[i].z << std::endl;

    //cloud_inとcloud_outをICPアルゴズムにより変換matrixを求める。
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    icp.setMaximumIterations (1000);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);

    print_info ("[done, "); print_value ("%d", Final.width * Final.height); print_info (" points], has converged: ");
    print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());

    //変換matrixを表示する
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    transformation_matrix = icp.getFinalTransformation ().cast<double>();
    print4x4Matrix (transformation_matrix);

    return (0);
}


/*
void
compute ()
{
    // Convert data to PointCloud<T>
    PointCloud<PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tgt (new PointCloud<PointXYZ>);
    // fromPCLPointCloud2 (*source, *src);
    // fromPCLPointCloud2 (*target, *tgt);

    // Fill in the src data
    src->width    = 10;
    src->height   = 1;
    src->is_dense = false;
    src->points.resize (src->width * src->height);

    for (size_t i = 0; i < src->points.size (); ++i)
    {
        src->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
        src->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
        src->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
    }

    std::cout << "Saved " << src->points.size () << " data points to input:"<< std::endl;
    for (size_t i = 0; i < src->points.size (); ++i) std::cout << "    " <<
    src->points[i].x << " " <<
    src->points[i].y << " " <<
    src->points[i].z << std::endl;

    *tgt = *src;

    for (size_t i = 0; i < tgt->points.size (); ++i)
    {
        tgt->points[i].x = src->points[i].x + 0.7;
        tgt->points[i].y = src->points[i].y + 0.5;
        tgt->points[i].z = src->points[i].z;
    }

    for (size_t i = 0; i < tgt->points.size (); ++i)
    std::cout << "    " <<
    tgt->points[i].x << " " <<
    tgt->points[i].y << " " <<
    tgt->points[i].z << std::endl;

    // Estimate
    TicToc tt;
    tt.tic ();


    IterativeClosestPoint<PointXYZ, PointXYZ, double> icp;
    //icp.setCorrespondenceEstimation (cens);
    //icp.setTransformationEstimation (te);
    //icp.addCorrespondenceRejector (cor_rej_o2o);
    icp.setInputSource (src);
    icp.setInputTarget (tgt);
    icp.setMaximumIterations (1000);
    icp.setTransformationEpsilon (1e-10);
    PointCloud<PointXYZ> output;
    icp.align (output);

    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points], has converged: ");
    print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
    Eigen::Matrix4d transformation = icp.getFinalTransformation ();
    std::cout << transformation << '\n';
    //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    PCL_ERROR ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
        transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
        transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
        transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
        transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));
}


int main (int argc, char** argv)
{
    compute ();

    return (0);
}

*/
