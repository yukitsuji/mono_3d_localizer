#ifndef ICP_7DOF_TRANSFORM_H_
#define ICP_7DOF_TRANSFORM_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/warp_point_rigid.h>
#include <pcl/registration/distances.h>

namespace pcl
{
  namespace registration
  {
    /** \brief TransformationEstimation7dof represents the base class for methods for non-rigid transformation estimation based on:
      *   - correspondence vectors
      *   - two point clouds (source and target) of the same size
      *   - a point cloud with a set of indices (source), and another point cloud (target)
      *   - two point clouds with two sets of indices (source and target) of the same size
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Yuki Tsuji
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename MatScalar = float>
    class TransformationEstimation7dof : public TransformationEstimation<PointSource, PointTarget, MatScalar>
    {
      public:
        typedef typename TransformationEstimation<PointSource, PointTarget, MatScalar>::Matrix4 Matrix4;
        typedef Eigen::Matrix<MatScalar, Eigen::Dynamic, 1> VectorX;
        typedef Eigen::Matrix<MatScalar, 4, 1> Vector4;        /** \brief Constructor. */

        TransformationEstimation7dof () {};

         /** \brief Destructor. */
        virtual ~TransformationEstimation7dof () {};

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] indices_src the vector of indices describing the points of interest in \a cloud_src
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] indices_tgt the vector of indices describing the correspondences of the interest points from
          * \a indices_src
          * \param[out] transformation_matrix the resultant transformation matrix
          */
        inline void
        estimateNonRigidTransformation (
            const pcl::PointCloud<PointSource> &cloud_src,
            const std::vector<int> &indices_src,
            const pcl::PointCloud<PointTarget> &cloud_tgt,
            const std::vector<int> &indices_tgt,
            Matrix4 &transformation_matrix) const;

        /** \brief Estimate a non-rigid rotation transformation between a source and a target point cloud using LM.
          * \param[in] cloud_src the source point cloud dataset
          * \param[in] cloud_tgt the target point cloud dataset
          * \param[in] correspondences the vector of correspondences between source and target point cloud
          * \param[out] transformation_matrix the resultant transformation matrix
          */
          inline void
          estimateNonRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              const pcl::Correspondences &correspondences,
              Matrix4 &transformation_matrix) const;


          inline void
          estimateRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              Matrix4 &transformation_matrix) const {};

          inline void
          estimateRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const std::vector<int> &indices_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              Matrix4 &transformation_matrix) const {};

          inline void
          estimateRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const std::vector<int> &indices_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              const std::vector<int> &indices_tgt,
              Matrix4 &transformation_matrix) const {};

          inline void
          estimateRigidTransformation (
              const pcl::PointCloud<PointSource> &cloud_src,
              const pcl::PointCloud<PointTarget> &cloud_tgt,
              const pcl::Correspondences &correspondences,
              Matrix4 &transformation_matrix) const {};

    };
  }
}

#endif /* ICP_7DOF_TRANSFORM_H_ */
