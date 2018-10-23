#include "icp_7dof/icp_7dof_correspondence_estimation.h"
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::registration::ICPCorrespondenceEstimationBase::setInputTarget (
    const PointCloudTargetConstPtr &cloud)
{
  // if (cloud->points.empty ())
  // {
  //   PCL_ERROR ("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
  //   return;
  // }
  // target_ = cloud;
  //
  // // Set the internal point representation of choice
  // if (point_representation_)
  //   tree_->setPointRepresentation (point_representation_);
  //
  // target_cloud_updated_ = true;
}

void pcl::registration::ICPCorrespondenceEstimationBase::setInputTargetTree (
    const PointCloudTargetConstPtr &cloud)
{
    target_ = cloud;

    // Set the internal point representation of choice
    if (point_representation_)
      tree_->setPointRepresentation (point_representation_);
    tree_->setInputCloud (cloud);
    target_cloud_updated_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::registration::ICPCorrespondenceEstimationBase::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::initCompute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  // If the target indices have been given via setIndicesTarget
  if (target_indices_)
    tree_->setInputCloud (target_, target_indices_);
  else
    tree_->setInputCloud (target_);

  target_cloud_updated_ = false;

  return (PCLBase<pcl::PointXYZ>::initCompute ());
}

bool
pcl::registration::ICPCorrespondenceEstimationBase::CustomInitCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::CustomInitCompute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  target_cloud_updated_ = false;

  return (PCLBase<pcl::PointXYZ>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::ICPCorrespondenceEstimation::determineCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
    if (!CustomInitCompute ())
        return;

    double max_dist_sqr = max_distance * max_distance;

    correspondences.resize (indices_->size ());

    std::vector<int> index (1);
    std::vector<float> distance (1);
    pcl::Correspondence corr;
    unsigned int nr_valid_correspondences = 0;

    // Check if the template types are the same. If true, avoid a copy.
    // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
        if (use_voxel_filter_)
        {
            if (voxel_grid_.searchFittedVoxel(input_->points[*idx])) {
                std::cout << "There are fitted voxel\n";
            }
            std::cout << "Use voxel grid filtering: \n";
        }

        tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
        // std::cout << "distance: " << distance[0] << ", max_dist_sqr: " << max_dist_sqr << ", max_distance: " << max_distance << "\n";
        if (distance[0] > max_dist_sqr)
            continue;

        corr.index_query = *idx;
        corr.index_match = index[0];
        corr.distance = distance[0];
        correspondences[nr_valid_correspondences++] = corr;
    }
    correspondences.resize (nr_valid_correspondences);
    deinitCompute ();
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::registration::ICPCorrespondenceEstimationBase::initComputeReciprocal ()
{
    exit(0);
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::ICPCorrespondenceEstimation::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
    exit(0);
}
