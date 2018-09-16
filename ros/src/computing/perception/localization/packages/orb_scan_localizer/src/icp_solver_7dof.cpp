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
#include <icp_7dof/icp_7dof.h>
#include <icp_7dof/icp_7dof_transform_lm.h>


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::registration;

Eigen::Vector4f    translation;
Eigen::Quaternionf orientation;

void
compute (const pcl::PCLPointCloud2::ConstPtr &source,
         const pcl::PCLPointCloud2::ConstPtr &target,
         pcl::PCLPointCloud2 &transformed_source)
{
    // Convert data to PointCloud<T>
    PointCloud<PointXYZ>::Ptr src (new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tgt (new PointCloud<PointXYZ>);
    fromPCLPointCloud2 (*source, *src);
    fromPCLPointCloud2 (*target, *tgt);

    // Estimate
    TicToc tt;
    tt.tic ();

    TransformationEstimation7dofLM<PointXYZ, PointXYZ, double>::Ptr te (new TransformationEstimation7dofLM<PointXYZ, PointXYZ, double>);
    // TransformationEstimationLM<PointXYZ, PointXYZ, double>::Ptr te (new TransformationEstimationLM<PointXYZ, PointXYZ, double>);
    CorrespondenceEstimation<PointXYZ, PointXYZ, double>::Ptr cens (new CorrespondenceEstimation<PointXYZ, PointXYZ, double>);
    cens->setInputSource (src);
    cens->setInputTarget (tgt);
    //cens->setSourceNormals (src);

    CorrespondenceRejectorOneToOne::Ptr cor_rej_o2o (new CorrespondenceRejectorOneToOne);

    // CorrespondenceRejectorMedianDistance::Ptr cor_rej_med (new CorrespondenceRejectorMedianDistance);
    // cor_rej_med->setInputSource<PointXYZ> (src);
    // cor_rej_med->setInputTarget<PointXYZ> (tgt);
    //icp.addCorrespondenceRejector (cor_rej_med);

    // CorrespondenceRejectorSampleConsensus<PointXYZ>::Ptr cor_rej_sac (new CorrespondenceRejectorSampleConsensus<PointXYZ>);
    // cor_rej_sac->setInputSource (src);
    // cor_rej_sac->setInputTarget (tgt);
    // cor_rej_sac->setInlierThreshold (0.005);
    // cor_rej_sac->setMaximumIterations (10000);
    //icp.addCorrespondenceRejector (cor_rej_sac);

    // CorrespondenceRejectorVarTrimmed::Ptr cor_rej_var (new CorrespondenceRejectorVarTrimmed);
    // cor_rej_var->setInputSource<PointXYZ> (src);
    // cor_rej_var->setInputTarget<PointXYZ> (tgt);
    //icp.addCorrespondenceRejector (cor_rej_var);

    // CorrespondenceRejectorTrimmed::Ptr cor_rej_tri (new CorrespondenceRejectorTrimmed);
    //icp.addCorrespondenceRejector (cor_rej_tri);

    IterativeClosestPoint7dof<PointXYZ, PointXYZ, double> icp;
    icp.setCorrespondenceEstimation (cens);
    icp.setTransformationEstimation (te);
    icp.addCorrespondenceRejector (cor_rej_o2o);
    icp.setInputSource (src);
    icp.setInputTarget (tgt);
    icp.setMaximumIterations (1000);
    icp.setTransformationEpsilon (1e-10);
    PointCloud<PointXYZ> output;
    icp.align (output);

    // print_info ("[done, "); print_value ("%g", tt.toc ()); print_info (" ms : "); print_value ("%d", output.width * output.height); print_info (" points], has converged: ");
    // print_value ("%d", icp.hasConverged ()); print_info (" with score: %f\n",  icp.getFitnessScore ());
    Eigen::Matrix4d transformation = icp.getFinalTransformation ();
    // //Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    // PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
    //     transformation (0, 0), transformation (0, 1), transformation (0, 2), transformation (0, 3),
    //     transformation (1, 0), transformation (1, 1), transformation (1, 2), transformation (1, 3),
    //     transformation (2, 0), transformation (2, 1), transformation (2, 2), transformation (2, 3),
    //     transformation (3, 0), transformation (3, 1), transformation (3, 2), transformation (3, 3));

    // Convert data back
    // pcl::PCLPointCloud2 output_source;
    // toPCLPointCloud2 (output, output_source);
    // concatenateFields (*source, output_source, transformed_source);
}


int main (int argc, char** argv)
{
    // Load the input files
    pcl::PCLPointCloud2::Ptr src (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr tgt (new pcl::PCLPointCloud2);

    // Perform the feature estimation
    pcl::PCLPointCloud2 output;
    compute (src, tgt, output);

    return (0);
}
