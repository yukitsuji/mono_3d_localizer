#include "icp_7dof/icp_7dof_correspondence_estimation.h"
#include <pcl/common/io.h>
#include <pcl/common/copy_point.h>

///////////////////////////////////////////////////////////////////////////////////////////
void pcl::registration::ICPCorrespondenceEstimationBase::setInputTarget (
    const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::registration::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_ = cloud;

  // Set the internal point representation of choice
  if (point_representation_)
    tree_->setPointRepresentation (point_representation_);

  target_cloud_updated_ = true;
}

void pcl::registration::ICPCorrespondenceEstimationBase::setInputTargetTree (
    const PointCloudTargetConstPtr &cloud)
{
    tree_->setInputCloud (cloud);
    target_cloud_updated_ = false;
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::registration::ICPCorrespondenceEstimationBase::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    // If the target indices have been given via setIndicesTarget
    if (target_indices_)
      tree_->setInputCloud (target_, target_indices_);
    else
      tree_->setInputCloud (target_);

    target_cloud_updated_ = false;
  }

  return (PCLBase<pcl::PointXYZ>::initCompute ());
}

bool
pcl::registration::ICPCorrespondenceEstimationBase::CustomInitCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    // If the target indices have been given via setIndicesTarget
    //if (target_indices_)
    //  tree_->setInputCloud (target_, target_indices_);
    //else
    //  tree_->setInputCloud (target_);

    target_cloud_updated_ = false;
  }

  return (PCLBase<pcl::PointXYZ>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
bool
pcl::registration::ICPCorrespondenceEstimationBase::initComputeReciprocal ()
{
  // Only update source kd-tree if a new target cloud was set
  if (source_cloud_updated_ && !force_no_recompute_reciprocal_)
  {
    if (point_representation_)
      tree_reciprocal_->setPointRepresentation (point_representation_);
    // If the target indices have been given via setIndicesTarget
    if (indices_)
      tree_reciprocal_->setInputCloud (getInputSource(), getIndicesSource());
    else
      tree_reciprocal_->setInputCloud (getInputSource());

    source_cloud_updated_ = false;
  }

  return (true);
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
  if (isSamePointType<pcl::PointXYZ, pcl::PointXYZ> ())
  {
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  else
  {
    pcl::PointXYZ pt;
    
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      // Copy the source data to a target pcl::PointXYZ format so we can search in the tree
      copyPoint (input_->points[*idx], pt);

      tree_->nearestKSearch (pt, 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::registration::ICPCorrespondenceEstimation::determineReciprocalCorrespondences (
    pcl::Correspondences &correspondences, double max_distance)
{
  if (!initCompute ())
    return;

  // setup tree for reciprocal search
  // Set the internal point representation of choice
  if (!initComputeReciprocal())
    return;
  double max_dist_sqr = max_distance * max_distance;

  correspondences.resize (indices_->size());
  std::vector<int> index (1);
  std::vector<float> distance (1);
  std::vector<int> index_reciprocal (1);
  std::vector<float> distance_reciprocal (1);
  pcl::Correspondence corr;
  unsigned int nr_valid_correspondences = 0;
  int target_idx = 0;

  // Check if the template types are the same. If true, avoid a copy.
  // Both point types MUST be registered using the POINT_CLOUD_REGISTER_POINT_STRUCT macro!
  if (isSamePointType<pcl::PointXYZ, pcl::PointXYZ> ())
  {
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      tree_->nearestKSearch (input_->points[*idx], 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      target_idx = index[0];

      tree_reciprocal_->nearestKSearch (target_->points[target_idx], 1, index_reciprocal, distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  else
  {
    pcl::PointXYZ pt_src;
    pcl::PointXYZ pt_tgt;
   
    // Iterate over the input set of source indices
    for (std::vector<int>::const_iterator idx = indices_->begin (); idx != indices_->end (); ++idx)
    {
      // Copy the source data to a target pcl::PointXYZ format so we can search in the tree
      copyPoint (input_->points[*idx], pt_src);

      tree_->nearestKSearch (pt_src, 1, index, distance);
      if (distance[0] > max_dist_sqr)
        continue;

      target_idx = index[0];

      // Copy the target data to a target pcl::PointXYZ format so we can search in the tree_reciprocal
      copyPoint (target_->points[target_idx], pt_tgt);

      tree_reciprocal_->nearestKSearch (pt_tgt, 1, index_reciprocal, distance_reciprocal);
      if (distance_reciprocal[0] > max_dist_sqr || *idx != index_reciprocal[0])
        continue;

      corr.index_query = *idx;
      corr.index_match = index[0];
      corr.distance = distance[0];
      correspondences[nr_valid_correspondences++] = corr;
    }
  }
  correspondences.resize (nr_valid_correspondences);
  deinitCompute ();
}


