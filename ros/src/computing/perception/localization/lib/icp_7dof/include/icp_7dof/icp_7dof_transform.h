#ifndef ICP_TRANSFORM_H_
#define ICP_TRANSFORM_H_

#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** \brief TransformationEstimation represents the base class for methods for transformation estimation based on:
      *   - correspondence vectors
      *   - two point clouds (source and target) of the same size
      *   - a point cloud with a set of indices (source), and another point cloud (target)
      *   - two point clouds with two sets of indices (source and target) of the same size
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Dirk Holz, Radu B. Rusu
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimation7dof
    {
      public:
        typedef Eigen::Matrix<Scalar, 4, 4> Matrix4;

        TransformationEstimation7dof () {};
        virtual ~TransformationEstimation7dof () {};

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a rigid rotation transformation between a source and a target point cloud.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src, const pcl::PointCloud<PointTarget> &cloud_tgt,
            const pcl::Correspondences &correspondences,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
          * \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        virtual void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix) const = 0;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
          virtual void
          estimateNonRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              const pcl::Correspondences &correspondences,
              Matrix4 &transformation_matrix) const = 0;

        typedef boost::shared_ptr<TransformationEstimation7dof<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimation7dof<PointSource, PointTarget, Scalar> > ConstPtr;
    };
  }
}

template class pcl::registration::TransformationEstimation7dof<pcl::PointXYZI, pcl::PointXYZI, double>;
template class pcl::registration::TransformationEstimation7dof<pcl::PointXYZI, pcl::PointXYZI, float>;
template class pcl::registration::TransformationEstimation7dof<pcl::PointXYZ, pcl::PointXYZ, double>;
template class pcl::registration::TransformationEstimation7dof<pcl::PointXYZ, pcl::PointXYZ, float>;
#endif
