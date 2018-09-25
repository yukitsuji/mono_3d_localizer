#ifndef IPC_7DOF_TRANSFORM_LM_HPP_
#define IPC_7DOF_TRANSFORM_LM_HPP_

#include "icp_7dof/warp_point_nonrigid_7d.h"
#include <pcl/registration/distances.h>
#include <unsupported/Eigen/NonLinearOptimization>

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar>
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::TransformationEstimation7dofLM ()
  : tmp_src_ ()
  , tmp_tgt_ ()
  , tmp_idx_src_ ()
  , tmp_idx_tgt_ ()
  , warp_point_ (new WarpPointRigid7D<PointSource, PointTarget, MatScalar>)
  , sigma_ (1.0)
{
};

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> void
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::estimateNonRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{

  // <cloud_src,cloud_src> is the source dataset
  if (cloud_src.points.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation] ");
    PCL_ERROR ("Number or points in source (%lu) differs than target (%lu)!\n",
               cloud_src.points.size (), cloud_tgt.points.size ());
    return;
  }
  if (cloud_src.points.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!\n",
               cloud_src.points.size ());
    return;
  }

  int n_unknowns = warp_point_->getDimension ();
  VectorX x (n_unknowns);
  x.setZero ();
  x[6] = 1.0;

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;

  OptimizationFunctor functor (static_cast<int> (cloud_src.points.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctor> num_diff (functor);
  //Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, double> lm (num_diff);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctor>, MatScalar> lm (num_diff);
  int info = lm.minimize (x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation]");
  PCL_DEBUG ("LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f", x[0]);
  for (int i = 1; i < n_unknowns; ++i)
    PCL_DEBUG (" %f", x[i]);
  PCL_DEBUG ("]\n");

  // Return the correct transformation
  warp_point_->setParam (x);
  transformation_matrix = warp_point_->getTransform ();

  tmp_src_ = NULL;
  tmp_tgt_ = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> void
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::estimateNonRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != cloud_tgt.points.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), cloud_tgt.points.size ());
    return;
  }

  // <cloud_src,cloud_src> is the source dataset
  transformation_matrix.setIdentity ();

  const int nr_correspondences = static_cast<const int> (cloud_tgt.points.size ());
  std::vector<int> indices_tgt;
  indices_tgt.resize(nr_correspondences);
  for (int i = 0; i < nr_correspondences; ++i)
    indices_tgt[i] = i;

  estimateNonRigidTransformation(cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> inline void
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::estimateNonRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const std::vector<int> &indices_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const std::vector<int> &indices_tgt,
    Matrix4 &transformation_matrix) const
{
  if (indices_src.size () != indices_tgt.size ())
  {
    PCL_ERROR ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation] Number or points in source (%lu) differs than target (%lu)!\n", indices_src.size (), indices_tgt.size ());
    return;
  }

  if (indices_src.size () < 4)     // need at least 4 samples
  {
    PCL_ERROR ("[pcl::IterativeClosestPointNonLinear::estimateNonRigidTransformationLM] ");
    PCL_ERROR ("Need at least 4 points to estimate a transform! Source and target have %lu points!",
               indices_src.size ());
    return;
  }

  int n_unknowns = warp_point_->getDimension ();  // get dimension of unknown space
  VectorX x (n_unknowns);
  x.setConstant (n_unknowns, 0);
  x[6] = 1.0;

  // Set temporary pointers
  tmp_src_ = &cloud_src;
  tmp_tgt_ = &cloud_tgt;
  tmp_idx_src_ = &indices_src;
  tmp_idx_tgt_ = &indices_tgt;

  OptimizationFunctorWithIndices functor (static_cast<int> (indices_src.size ()), this);
  Eigen::NumericalDiff<OptimizationFunctorWithIndices> num_diff (functor);
  //Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithIndices> > lm (num_diff);
  Eigen::LevenbergMarquardt<Eigen::NumericalDiff<OptimizationFunctorWithIndices>, MatScalar> lm (num_diff);
  int info = lm.minimize (x);

  // Compute the norm of the residuals
  PCL_DEBUG ("[pcl::registration::TransformationEstimation7dofLM::estimateNonRigidTransformation] LM solver finished with exit code %i, having a residual norm of %g. \n", info, lm.fvec.norm ());
  PCL_DEBUG ("Final solution: [%f", x[0]);
  for (int i = 1; i < n_unknowns; ++i)
    PCL_DEBUG (" %f", x[i]);
  PCL_DEBUG ("]\n");

  // Return the correct transformation
  warp_point_->setParam (x);
  transformation_matrix = warp_point_->getTransform ();

  tmp_src_ = NULL;
  tmp_tgt_ = NULL;
  tmp_idx_src_ = tmp_idx_tgt_ = NULL;
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> inline void
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::estimateNonRigidTransformation (
    const pcl::PointCloud<PointSource> &cloud_src,
    const pcl::PointCloud<PointTarget> &cloud_tgt,
    const pcl::Correspondences &correspondences,
    Matrix4 &transformation_matrix) const
{
  const int nr_correspondences = static_cast<const int> (correspondences.size ());
  std::vector<int> indices_src (nr_correspondences);
  std::vector<int> indices_tgt (nr_correspondences);
  for (int i = 0; i < nr_correspondences; ++i)
  {
    indices_src[i] = correspondences[i].index_query;
    indices_tgt[i] = correspondences[i].index_match;
  }

  estimateNonRigidTransformation (cloud_src, indices_src, cloud_tgt, indices_tgt, transformation_matrix);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> int
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::OptimizationFunctor::operator () (
    const VectorX &x, VectorX &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;

  // Initialize the warp function with the given parameters
  estimator_->warp_point_->setParam (x);

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < values (); ++i)
  {
    const PointSource & p_src = src_points.points[i];
    const PointTarget & p_tgt = tgt_points.points[i];

    // Transform the source point based on the current warp parameters
    Vector4 p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);

    // Estimate the distance (cost function)
    fvec[i] = estimator_->computeDistance (p_src_warped, p_tgt);
  }
  return (0);
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename MatScalar> int
pcl::registration::TransformationEstimation7dofLM<PointSource, PointTarget, MatScalar>::OptimizationFunctorWithIndices::operator() (
    const VectorX &x, VectorX &fvec) const
{
  const PointCloud<PointSource> & src_points = *estimator_->tmp_src_;
  const PointCloud<PointTarget> & tgt_points = *estimator_->tmp_tgt_;
  const std::vector<int> & src_indices = *estimator_->tmp_idx_src_;
  const std::vector<int> & tgt_indices = *estimator_->tmp_idx_tgt_;

  // Initialize the warp function with the given parameters
  estimator_->warp_point_->setParam (x);

  // Transform each source point and compute its distance to the corresponding target point
  for (int i = 0; i < values (); ++i)
  {
    const PointSource & p_src = src_points.points[src_indices[i]];
    const PointTarget & p_tgt = tgt_points.points[tgt_indices[i]];

    // Transform the source point based on the current warp parameters
    Vector4 p_src_warped;
    estimator_->warp_point_->warpPoint (p_src, p_src_warped);

    // Estimate the distance (cost function)
    fvec[i] = estimator_->computeDistance (p_src_warped, p_tgt);
  }
  return (0);
}

//template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZI, pcl::PointXYZI, double>;
//template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZI, pcl::PointXYZI, float>;
//template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZ, pcl::PointXYZ, double>;
//template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZ, pcl::PointXYZ, float>;

//#define PCL_INSTANTIATE_TransformationEstimation7dofLM(T,U) template class PCL_EXPORTS pcl::registration::TransformationEstimation7dofLM<T,U>;
#endif //IPC_7DOF_TRANSFORM_LM_HPP_
