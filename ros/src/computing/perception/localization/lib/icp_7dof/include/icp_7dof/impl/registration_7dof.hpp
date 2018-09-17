///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::Registration7dof<PointSource, PointTarget, Scalar>::setInputTarget (const PointCloudTargetConstPtr &cloud)
{
  if (cloud->points.empty ())
  {
    PCL_ERROR ("[pcl::%s::setInputTarget] Invalid or empty point cloud dataset given!\n", getClassName ().c_str ());
    return;
  }
  target_ = cloud;
  target_cloud_updated_ = true;
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::Registration7dof<PointSource, PointTarget, Scalar>::initCompute ()
{
  if (!target_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input target dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  // Only update target kd-tree if a new target cloud was set
  if (target_cloud_updated_ && !force_no_recompute_)
  {
    tree_->setInputCloud (target_);
    target_cloud_updated_ = false;
  }


  // Update the correspondence estimation
  if (correspondence_estimation_)
  {
    correspondence_estimation_->setSearchMethodTarget (tree_, force_no_recompute_);
    correspondence_estimation_->setSearchMethodSource (tree_reciprocal_, force_no_recompute_reciprocal_);
  }

  // Note: we /cannot/ update the search method on all correspondence rejectors, because we know
  // nothing about them. If they should be cached, they must be cached individually.

  return (PCLBase<PointSource>::initCompute ());
}

///////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> bool
pcl::Registration7dof<PointSource, PointTarget, Scalar>::initComputeReciprocal ()
{
  if (!input_)
  {
    PCL_ERROR ("[pcl::registration::%s::compute] No input source dataset was given!\n", getClassName ().c_str ());
    return (false);
  }

  if (source_cloud_updated_ && !force_no_recompute_reciprocal_)
  {
    tree_reciprocal_->setInputCloud (input_);
    source_cloud_updated_ = false;
  }
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline double
pcl::Registration7dof<PointSource, PointTarget, Scalar>::getFitnessScore (
    const std::vector<float> &distances_a,
    const std::vector<float> &distances_b)
{
  unsigned int nr_elem = static_cast<unsigned int> (std::min (distances_a.size (), distances_b.size ()));
  Eigen::VectorXf map_a = Eigen::VectorXf::Map (&distances_a[0], nr_elem);
  Eigen::VectorXf map_b = Eigen::VectorXf::Map (&distances_b[0], nr_elem);
  return (static_cast<double> ((map_a - map_b).sum ()) / static_cast<double> (nr_elem));
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline double
pcl::Registration7dof<PointSource, PointTarget, Scalar>::getFitnessScore (double max_range)
{

  double fitness_score = 0.0;

  // Transform the input dataset using the final transformation
  PointCloudSource input_transformed;
  transformPointCloud (*input_, input_transformed, final_transformation_);

  std::vector<int> nn_indices (1);
  std::vector<float> nn_dists (1);

  // For each point in the source dataset
  int nr = 0;
  for (size_t i = 0; i < input_transformed.points.size (); ++i)
  {
    // Find its nearest neighbor in the target
    tree_->nearestKSearch (input_transformed.points[i], 1, nn_indices, nn_dists);

    // Deal with occlusions (incomplete targets)
    if (nn_dists[0] <= max_range)
    {
      // Add to the fitness score
      fitness_score += nn_dists[0];
      nr++;
    }
  }

  if (nr > 0)
    return (fitness_score / nr);
  else
    return (std::numeric_limits<double>::max ());

}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::Registration7dof<PointSource, PointTarget, Scalar>::align (PointCloudSource &output)
{
  align (output, Matrix4::Identity ());
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar> inline void
pcl::Registration7dof<PointSource, PointTarget, Scalar>::align (PointCloudSource &output, const Matrix4& guess)
{
  if (!initCompute ())
    return;

  // Resize the output dataset
  if (output.points.size () != indices_->size ())
    output.points.resize (indices_->size ());
  // Copy the header
  output.header   = input_->header;
  // Check if the output will be computed for all points or only a subset
  if (indices_->size () != input_->points.size ())
  {
    output.width    = static_cast<uint32_t> (indices_->size ());
    output.height   = 1;
  }
  else
  {
    output.width    = static_cast<uint32_t> (input_->width);
    output.height   = input_->height;
  }
  output.is_dense = input_->is_dense;

  // Copy the point data to output
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i] = input_->points[(*indices_)[i]];

  // Set the internal point representation of choice unless otherwise noted
  if (point_representation_ && !force_no_recompute_)
    tree_->setPointRepresentation (point_representation_);

  // Perform the actual transformation computation
  converged_ = false;
  final_transformation_ = transformation_ = previous_transformation_ = Matrix4::Identity ();

  // Right before we estimate the transformation, we set all the point.data[3] values to 1 to aid the rigid
  // transformation
  for (size_t i = 0; i < indices_->size (); ++i)
    output.points[i].data[3] = 1.0;

  computeTransformation (output, guess);

  deinitCompute ();
}
