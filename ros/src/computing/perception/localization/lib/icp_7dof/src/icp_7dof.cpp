//#include <pcl/registration/boost.h>
//#include <pcl/correspondence.h>
#include "icp_7dof/icp_7dof.h"
#include "icp_7dof/icp_7dof_correspondence_estimation.h"
#include <chrono>


void
pcl::IterativeClosestPoint7dof::transformCloudPublic (
    const PointCloudSource &input,
    PointCloudSource &output,
    const Matrix4 &transform)
{
  Eigen::Vector4f pt (0.0f, 0.0f, 0.0f, 1.0f), pt_t;
  Eigen::Matrix4f tr = transform.template cast<float> ();

  // XYZ is ALWAYS present due to the templatization, so we only have to check for normals
  if (source_has_normals_)
  {
    Eigen::Vector3f nt, nt_t;
    Eigen::Matrix3f rot = tr.block<3, 3> (0, 0);

    for (size_t i = 0; i < input.size (); ++i)
    {
      const uint8_t* data_in = reinterpret_cast<const uint8_t*> (&input[i]);
      uint8_t* data_out = reinterpret_cast<uint8_t*> (&output[i]);
      memcpy (&pt[0], data_in + x_idx_offset_, sizeof (float));
      memcpy (&pt[1], data_in + y_idx_offset_, sizeof (float));
      memcpy (&pt[2], data_in + z_idx_offset_, sizeof (float));

      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy (data_out + x_idx_offset_, &pt_t[0], sizeof (float));
      memcpy (data_out + y_idx_offset_, &pt_t[1], sizeof (float));
      memcpy (data_out + z_idx_offset_, &pt_t[2], sizeof (float));

      memcpy (&nt[0], data_in + nx_idx_offset_, sizeof (float));
      memcpy (&nt[1], data_in + ny_idx_offset_, sizeof (float));
      memcpy (&nt[2], data_in + nz_idx_offset_, sizeof (float));

      if (!pcl_isfinite (nt[0]) || !pcl_isfinite (nt[1]) || !pcl_isfinite (nt[2]))
        continue;

      nt_t = rot * nt;

      memcpy (data_out + nx_idx_offset_, &nt_t[0], sizeof (float));
      memcpy (data_out + ny_idx_offset_, &nt_t[1], sizeof (float));
      memcpy (data_out + nz_idx_offset_, &nt_t[2], sizeof (float));
    }
  }
  else
  {
    for (size_t i = 0; i < input.size (); ++i)
    {
      const uint8_t* data_in = reinterpret_cast<const uint8_t*> (&input[i]);
      uint8_t* data_out = reinterpret_cast<uint8_t*> (&output[i]);
      memcpy (&pt[0], data_in + x_idx_offset_, sizeof (float));
      memcpy (&pt[1], data_in + y_idx_offset_, sizeof (float));
      memcpy (&pt[2], data_in + z_idx_offset_, sizeof (float));

      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy (data_out + x_idx_offset_, &pt_t[0], sizeof (float));
      memcpy (data_out + y_idx_offset_, &pt_t[1], sizeof (float));
      memcpy (data_out + z_idx_offset_, &pt_t[2], sizeof (float));
    }
  }

}

///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::IterativeClosestPoint7dof::transformCloud (
    const PointCloudSource &input,
    PointCloudSource &output,
    const Matrix4 &transform)
{
  Eigen::Vector4f pt (0.0f, 0.0f, 0.0f, 1.0f), pt_t;
  Eigen::Matrix4f tr = transform.template cast<float> ();

  // XYZ is ALWAYS present due to the templatization, so we only have to check for normals
  if (source_has_normals_)
  {
    Eigen::Vector3f nt, nt_t;
    Eigen::Matrix3f rot = tr.block<3, 3> (0, 0);

    for (size_t i = 0; i < input.size (); ++i)
    {
      const uint8_t* data_in = reinterpret_cast<const uint8_t*> (&input[i]);
      uint8_t* data_out = reinterpret_cast<uint8_t*> (&output[i]);
      memcpy (&pt[0], data_in + x_idx_offset_, sizeof (float));
      memcpy (&pt[1], data_in + y_idx_offset_, sizeof (float));
      memcpy (&pt[2], data_in + z_idx_offset_, sizeof (float));

      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy (data_out + x_idx_offset_, &pt_t[0], sizeof (float));
      memcpy (data_out + y_idx_offset_, &pt_t[1], sizeof (float));
      memcpy (data_out + z_idx_offset_, &pt_t[2], sizeof (float));

      memcpy (&nt[0], data_in + nx_idx_offset_, sizeof (float));
      memcpy (&nt[1], data_in + ny_idx_offset_, sizeof (float));
      memcpy (&nt[2], data_in + nz_idx_offset_, sizeof (float));

      if (!pcl_isfinite (nt[0]) || !pcl_isfinite (nt[1]) || !pcl_isfinite (nt[2]))
        continue;

      nt_t = rot * nt;

      memcpy (data_out + nx_idx_offset_, &nt_t[0], sizeof (float));
      memcpy (data_out + ny_idx_offset_, &nt_t[1], sizeof (float));
      memcpy (data_out + nz_idx_offset_, &nt_t[2], sizeof (float));
    }
  }
  else
  {
    for (size_t i = 0; i < input.size (); ++i)
    {
      const uint8_t* data_in = reinterpret_cast<const uint8_t*> (&input[i]);
      uint8_t* data_out = reinterpret_cast<uint8_t*> (&output[i]);
      memcpy (&pt[0], data_in + x_idx_offset_, sizeof (float));
      memcpy (&pt[1], data_in + y_idx_offset_, sizeof (float));
      memcpy (&pt[2], data_in + z_idx_offset_, sizeof (float));

      if (!pcl_isfinite (pt[0]) || !pcl_isfinite (pt[1]) || !pcl_isfinite (pt[2]))
        continue;

      pt_t = tr * pt;

      memcpy (data_out + x_idx_offset_, &pt_t[0], sizeof (float));
      memcpy (data_out + y_idx_offset_, &pt_t[1], sizeof (float));
      memcpy (data_out + z_idx_offset_, &pt_t[2], sizeof (float));
    }
  }
}


///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::IterativeClosestPoint7dof::computeTransformation (
    PointCloudSource &output, const Matrix4 &guess, const Matrix4 &global_to_local)
{
  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudSourcePtr input_transformed (new PointCloudSource);

  nr_iterations_ = 0;
  converged_ = false;

  const bool debug = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // If the guessed transformation is non identity
  //if (guess != Matrix4::Identity ())
  //{
  //  input_transformed->resize (input_->size ());
  //   // Apply guessed transformation prior to search for neighbours
  //  transformCloud (*input_, *input_transformed, guess);
  //}
  //else

  *input_transformed = *input_;

  transformation_ = Matrix4::Identity ();

  // Pass in the default target for the Correspondence Estimation/Rejection code
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  correspondence_estimation_->setInputTarget (target_);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  if (debug)
      std::cout << "setInputTarget: " << ttrack << "\n";

  convergence_criteria_->setMaximumIterations (max_iterations_);
  convergence_criteria_->setRelativeMSE (euclidean_fitness_epsilon_);
  convergence_criteria_->setTranslationThreshold (transformation_epsilon_);
  // if (transformation_rotation_epsilon_ > 0)
  //   convergence_criteria_->setRotationThreshold (transformation_rotation_epsilon_);
  // else
  convergence_criteria_->setRotationThreshold (1.0 - transformation_epsilon_);

  Eigen::Matrix4d local_to_global = global_to_local.inverse();

  // Repeat until convergence
  do
  {
      previous_transformation_ = transformation_;

      // Set the source each iteration, to ensure the dirty flag is updated
      t1 = std::chrono::steady_clock::now();
      correspondence_estimation_->setInputSource (input_transformed);
      t2 = std::chrono::steady_clock::now();
      ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      if (debug)
          std::cout << "setInputSource: " << ttrack << "\n";

      t1 = std::chrono::steady_clock::now();
      // Estimate correspondences
      correspondence_estimation_->determineCorrespondences (*correspondences_, corr_dist_threshold_);

      t2 = std::chrono::steady_clock::now();
      ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      if (debug)
      {
          std::cout << "determineCorrespondences: " << ttrack << "\n";
          std::cout << "correspondence size: from " << input_transformed->size() << " to " << correspondences_->size() << "\n";
      }

      int cnt = static_cast<int>(correspondences_->size());
      // Check whether we have enough correspondences
      if (cnt < min_number_correspondences_)
      {
          PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters. %d \n", getClassName ().c_str (), corr_dist_threshold_);
          convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<double>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
          converged_ = false;
          break;
      }


      t1 = std::chrono::steady_clock::now();
      // customize input and its correspondence
      // computational cost would be explosive if the transformation from global to local is done in estimateNonRigidTransformation
      // that should be done before estimateNonRigidTransformation.
      PointCloudSourcePtr local_input (new PointCloudSource);
      PointCloudSourcePtr local_selected_input (new PointCloudSource);
      PointCloudSourcePtr local_target (new PointCloudSource);

      *local_input = *input_transformed;
      transformCloudPublic(*local_input, *local_input, global_to_local);

      local_selected_input->resize(cnt);
      local_target->resize(cnt);
      for (int i=0; i<cnt; ++i)
      {
          local_selected_input->points[i] = local_input->points[(*correspondences_)[i].index_query];
          local_target->points[i] = target_->points[(*correspondences_)[i].index_match];
      }

      transformCloudPublic(*local_target, *local_target, global_to_local);

      t2 = std::chrono::steady_clock::now();
      ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      if (debug)
          std::cout << "global to local: " << ttrack << "\n";

      t1 = std::chrono::steady_clock::now();
      // Estimate the transform
      // transformation_estimation_->estimateNonRigidTransformation (*input_transformed, *target_, *correspondences_, transformation_);
      transformation_estimation_->estimateNonRigidTransformation (*local_selected_input, *local_target, transformation_);
      t2 = std::chrono::steady_clock::now();
      ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      if (debug)
          std::cout << "estimateNonRigidTransformation: " << ttrack << "\n";

      // Transform the data
      t1 = std::chrono::steady_clock::now();
      // transformCloud (*input_transformed, *input_transformed, transformation_);

      transformCloud (*local_input, *local_input, transformation_);
      *input_transformed = *local_input;
      transformCloud (*input_transformed, *input_transformed, local_to_global);

      t2 = std::chrono::steady_clock::now();
      ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
      if (debug)
          std::cout << "transformCloud: " << ttrack << "\n";

      // Obtain the final transformation
      final_transformation_ = transformation_ * final_transformation_;

      if (debug)
          std::cout << "transformation_\n" << final_transformation_ << "\n";

      ++nr_iterations_;

      converged_ = static_cast<bool> ((*convergence_criteria_));
  }
  while (!converged_);

  std::cout << "transformation_\n";
  std::cout << final_transformation_ << "\n";

  // Transform the input cloud using the final transformation
  PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
      final_transformation_ (0, 0), final_transformation_ (0, 1), final_transformation_ (0, 2), final_transformation_ (0, 3),
      final_transformation_ (1, 0), final_transformation_ (1, 1), final_transformation_ (1, 2), final_transformation_ (1, 3),
      final_transformation_ (2, 0), final_transformation_ (2, 1), final_transformation_ (2, 2), final_transformation_ (2, 3),
      final_transformation_ (3, 0), final_transformation_ (3, 1), final_transformation_ (3, 2), final_transformation_ (3, 3));

  // Copy all the values
  output = *input_;
  Eigen::Matrix4d transformation = local_to_global * final_transformation_ * global_to_local;
  transformCloudPublic(output, output, transformation);
}


///////////////////////////////////////////////////////////////////////////////////////////
void
pcl::IterativeClosestPoint7dof::computeTransformation (
    PointCloudSource &output, const Matrix4 &guess)
{
  // Point cloud containing the correspondences of each point in <input, indices>
  PointCloudSourcePtr input_transformed (new PointCloudSource);

  nr_iterations_ = 0;
  converged_ = false;

  const bool debug = false;

  // Initialise final transformation to the guessed one
  final_transformation_ = guess;

  // If the guessed transformation is non identity
  if (guess != Matrix4::Identity ())
  {
    input_transformed->resize (input_->size ());
     // Apply guessed transformation prior to search for neighbours
    transformCloud (*input_, *input_transformed, guess);
  }
  else
    *input_transformed = *input_;

  transformation_ = Matrix4::Identity ();

  // Make blobs if necessary
  determineRequiredBlobData ();
  PCLPointCloud2::Ptr target_blob (new PCLPointCloud2);
  if (need_target_blob_)
    pcl::toPCLPointCloud2 (*target_, *target_blob);

  // Pass in the default target for the Correspondence Estimation/Rejection code
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
  correspondence_estimation_->setInputTarget (target_);
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
  double ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
  if (debug) {
      std::cout << "setInputTarget: " << ttrack << "\n";
      std::cout << "correspondence_estimation_->requiresTargetNormals: " << correspondence_estimation_->requiresTargetNormals() << "\n";
  }

  if (correspondence_estimation_->requiresTargetNormals ())
      correspondence_estimation_->setTargetNormals (target_blob);


  convergence_criteria_->setMaximumIterations (max_iterations_);
  convergence_criteria_->setRelativeMSE (euclidean_fitness_epsilon_);
  convergence_criteria_->setTranslationThreshold (transformation_epsilon_);
  // if (transformation_rotation_epsilon_ > 0)
  //   convergence_criteria_->setRotationThreshold (transformation_rotation_epsilon_);
  // else
  convergence_criteria_->setRotationThreshold (1.0 - transformation_epsilon_);

  // Repeat until convergence
  do
  {
    // Get blob data if needed
    PCLPointCloud2::Ptr input_transformed_blob;
    if (need_source_blob_)
    {
      input_transformed_blob.reset (new PCLPointCloud2);
      toPCLPointCloud2 (*input_transformed, *input_transformed_blob);
    }
    // Save the previously estimated transformation
    previous_transformation_ = transformation_;

    // Set the source each iteration, to ensure the dirty flag is updated
    t1 = std::chrono::steady_clock::now();
    correspondence_estimation_->setInputSource (input_transformed);
    t2 = std::chrono::steady_clock::now();
    ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (debug)
        std::cout << "setInputSource: " << ttrack << "\n";

    t1 = std::chrono::steady_clock::now();
    // Estimate correspondences
    correspondence_estimation_->determineCorrespondences (*correspondences_, corr_dist_threshold_);

    t2 = std::chrono::steady_clock::now();
    ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (debug)
    {
        std::cout << "determineCorrespondences: " << ttrack << "\n";
        std::cout << "correspondence size: from " << input_transformed->size() << " to " << correspondences_->size() << "\n";
    }

    size_t cnt = correspondences_->size ();
    // Check whether we have enough correspondences
    if (static_cast<int> (cnt) < min_number_correspondences_)
    {
      PCL_ERROR ("[pcl::%s::computeTransformation] Not enough correspondences found. Relax your threshold parameters. %d \n", getClassName ().c_str (), corr_dist_threshold_);
      convergence_criteria_->setConvergenceState(pcl::registration::DefaultConvergenceCriteria<double>::CONVERGENCE_CRITERIA_NO_CORRESPONDENCES);
      converged_ = false;
      break;
    }

    t1 = std::chrono::steady_clock::now();
    // Estimate the transform
    transformation_estimation_->estimateNonRigidTransformation (*input_transformed, *target_, *correspondences_, transformation_);
    t2 = std::chrono::steady_clock::now();
    ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (debug)
        std::cout << "estimateNonRigidTransformation: " << ttrack << "\n";

    // Transform the data
    t1 = std::chrono::steady_clock::now();
    transformCloud (*input_transformed, *input_transformed, transformation_);
    t2 = std::chrono::steady_clock::now();
    ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
    if (debug)
        std::cout << "transformCloud: " << ttrack << "\n";

    // Obtain the final transformation
    final_transformation_ = transformation_ * final_transformation_;

    ++nr_iterations_;

    converged_ = static_cast<bool> ((*convergence_criteria_));
  }
  while (!converged_);

  std::cout << "transformation_\n";
  std::cout << final_transformation_ << "\n";

  // Transform the input cloud using the final transformation
  PCL_DEBUG ("Transformation is:\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n\t%5f\t%5f\t%5f\t%5f\n",
      final_transformation_ (0, 0), final_transformation_ (0, 1), final_transformation_ (0, 2), final_transformation_ (0, 3),
      final_transformation_ (1, 0), final_transformation_ (1, 1), final_transformation_ (1, 2), final_transformation_ (1, 3),
      final_transformation_ (2, 0), final_transformation_ (2, 1), final_transformation_ (2, 2), final_transformation_ (2, 3),
      final_transformation_ (3, 0), final_transformation_ (3, 1), final_transformation_ (3, 2), final_transformation_ (3, 3));

  // Copy all the values
  output = *input_;
  // Transform the XYZ + normals
  transformCloud (*input_, output, final_transformation_);
}

void
pcl::IterativeClosestPoint7dof::determineRequiredBlobData ()
{
  need_source_blob_ = false;
  need_target_blob_ = false;
  // Check estimator
  need_source_blob_ |= correspondence_estimation_->requiresSourceNormals ();
  need_target_blob_ |= correspondence_estimation_->requiresTargetNormals ();
  // Add warnings if necessary
  if (correspondence_estimation_->requiresSourceNormals () && !source_has_normals_)
  {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects source normals, but we can't provide them.\n", getClassName ().c_str ());
  }
  if (correspondence_estimation_->requiresTargetNormals () && !target_has_normals_)
  {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Estimator expects target normals, but we can't provide them.\n", getClassName ().c_str ());
  }
  // Check rejectors
  for (size_t i = 0; i < correspondence_rejectors_.size (); i++)
  {
    registration::CorrespondenceRejector::Ptr& rej = correspondence_rejectors_[i];
    need_source_blob_ |= rej->requiresSourcePoints ();
    need_source_blob_ |= rej->requiresSourceNormals ();
    need_target_blob_ |= rej->requiresTargetPoints ();
    need_target_blob_ |= rej->requiresTargetNormals ();
    if (rej->requiresSourceNormals () && !source_has_normals_)
    {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects source normals, but we can't provide them.\n", getClassName ().c_str (), rej->getClassName ().c_str ());
    }
    if (rej->requiresTargetNormals () && !target_has_normals_)
    {
      PCL_WARN("[pcl::%s::determineRequiredBlobData] Rejector %s expects target normals, but we can't provide them.\n", getClassName ().c_str (), rej->getClassName ().c_str ());
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////

//template class pcl::IterativeClosestPoint7dof;
//template class pcl::IterativeClosestPoint7dof<pcl::PointXYZ, pcl::PointXYZ, float>;
//template class pcl::IterativeClosestPoint7dof<pcl::PointXYZI, pcl::PointXYZI, double>;
//template class pcl::IterativeClosestPoint7dof<pcl::PointXYZI, pcl::PointXYZI, float>;

//template class pcl::IterativeClosestPoint7dofWithNormals<pcl::PointNormal, pcl::PointNormal, double>;
//template class pcl::IterativeClosestPoint7dofWithNormals<pcl::PointNormal, pcl::PointNormal, double>;
