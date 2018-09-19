#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/console/parse.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
//#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/correspondence_rejection_trimmed.h>
#include <pcl/registration/correspondence_rejection_var_trimmed.h>
#include <icp_7dof/icp_7dof.h>
#include <icp_7dof/icp_7dof_transform_lm.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
compute ()
{
    //fromPCLPointCloud2 (*source, *src);
    //fromPCLPointCloud2 (*target, *tgt);
    //点群を宣言する。
    pcl::PointCloud<pcl::PointXYZ>::Ptr src (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgt (new pcl::PointCloud<pcl::PointXYZ>);

    //点群の移動量を設定
    double x_offset=0.7f;
    double y_offset=0.5f;

    //点群srcにデータを入力
    src->width    = 10;
    src->height   = 1;
    src->is_dense = false;
    src->points.resize (src->width * src->height);

    for (size_t i = 0; i < src->points.size (); ++i)   {
        src->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
        src->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
        src->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f) * 10;
    }

    // std::cout << "Saved " << src->points.size () << " data points to input:"<< std::endl;
    for (size_t i = 0; i < src->points.size (); ++i) std::cout << "    " <<
    src->points[i].x << " " <<
    src->points[i].y << " " <<
    src->points[i].z << std::endl;
    *tgt = *src;
    std::cout << "size:" << tgt->points.size() << std::endl;

    //点群を移動させtgtを作る。
    for (size_t i = 0; i < src->points.size (); ++i){
        tgt->points[i].x = 2.1 * (src->points[i].x + 1);// + x_offset; //+ 102 * rand () / (RAND_MAX + 1.0f);
        tgt->points[i].y = 2.1 * (src->points[i].y + 2);// + y_offset; //+ 102 * rand () / (RAND_MAX + 1.0f);
        tgt->points[i].z = 2.1 * (src->points[i].z + 2);// + y_offset; //+ 102 * rand () / (RAND_MAX + 1.0f);
    }
    // std::cout << "Transformed " << src->points.size () << " data points:"  << std::endl;

    for (size_t i = 0; i < tgt->points.size (); ++i)
    std::cout << "    " <<
    tgt->points[i].x << " " <<
    tgt->points[i].y << " " <<
    tgt->points[i].z << std::endl;

    // Estimate
    TicToc tt;
    tt.tic ();

    // TransformationEstimation7dofLM<PointXYZ, PointXYZ, double>::Ptr te (new TransformationEstimation7dofLM<PointXYZ, PointXYZ, double>);
    // CorrespondenceEstimation<PointXYZ, PointXYZ, double>::Ptr cens (new CorrespondenceEstimation<PointXYZ, PointXYZ, double>);
    // cens->setInputSource (src);
    // cens->setInputTarget (tgt);
    //cens->setSourceNormals (src);

    // CorrespondenceRejectorDistance::Ptr cor_rej_o2o (new CorrespondenceRejectorDistance);
    CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new CorrespondenceRejectorOneToOne);

    // CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new CorrespondenceRejectorMedianDistance);
    // cor_rej_med->setInputSource<PointXYZ> (src);
    // cor_rej_med->setInputTarget<PointXYZ> (tgt);
    //icp.addCorrespondenceRejector (cor_rej_med);

    IterativeClosestPoint7dof<PointXYZ, PointXYZ, double> icp;
    // icp.setCorrespondenceEstimation (cens);
    // icp.setTransformationEstimation (te);
    icp.addCorrespondenceRejector (cor_rej_o2o);
    icp.setInputSource (src);
    icp.setInputTarget (tgt);
    icp.setMaximumIterations (1000);
    icp.setTransformationEpsilon (1e-10);
    //icp.convergence_criteria_->setMaximumIterationsSimilarTransforms(2);
    PointCloud<PointXYZ> output;
    icp.align (output);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points], has converged: ");
    print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
    Eigen::Matrix4d transformation = icp.getFinalTransformation ();
    PCL_ERROR ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
        transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
        transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
        transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
        transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));

    for (size_t i = 0; i < output.points.size (); ++i) std::cout << "    " <<
    output.points[i].x << " " <<
    output.points[i].y << " " <<
    output.points[i].z << std::endl;
    std::cout << "size:" << tgt->points.size() << std::endl;

    for (size_t i = 0; i < tgt->points.size (); ++i)
    std::cout << "    " <<
    tgt->points[i].x << " " <<
    tgt->points[i].y << " " <<
    tgt->points[i].z << std::endl;
    /*
    icp.setInputSource (src);
    icp.setInputTarget (tgt);
    icp.setMaximumIterations (1000);
    icp.setTransformationEpsilon (1e-10);
    //icp.convergence_criteria_->setMaximumIterationsSimilarTransforms(2);
    PointCloud<PointXYZ> output2;
    icp.align (output2, transformation);
    print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output2.width * output2.height); print_info (" points], has converged: ");
    print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
    transformation = icp.getFinalTransformation ();
    PCL_ERROR ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
        transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
        transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
        transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
        transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));
    */
}


int main (int argc, char** argv)
{
    setVerbosityLevel(L_DEBUG);
    // Load the input files
    compute ();

    return (0);
}
