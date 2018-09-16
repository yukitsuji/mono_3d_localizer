
#ifndef ICP_7dof_TRANSFORM_ESTIMATION_H_
#define ICP_7dof_TRANSFORM_ESTIMATION_H_

#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** @b TransformationEstimation7dof uses Levenberg Marquardt optimization to find the
      * transformation that minimizes the point-to-point distance between the given correspondences.
      * Distance metrics: huber
      * Error metrics: point-to-point
      *
      * \author Yuki Tsuji
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename double = float>
    class TransformationEstimation7dof : public TransformationEstimationLM<PointSource, PointTarget, double>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimation7dof<PointSource, PointTarget, double> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimation7dof<PointSource, PointTarget, double> > ConstPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;

        typedef Eigen::Matrix<double, 4, 1> Vector4;

        TransformationEstimation7dof () {};
        virtual ~TransformationEstimation7dof () {};

      protected:
        virtual double
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt, const double sigma) const
        {
          const Vector4 s (p_src.x, p_src.y, p_src.z, 0);
          const Vector4 t (p_tgt.x, p_tgt.y, p_tgt.z, 0);
          return huber(s, t, sigma);
        }

        virtual double
        computeDistance (const Vector4 &p_src, const PointTarget &p_tgt, const double sigma) const
        {
          return huber(s, t, sigma);
        }

    };
  }
}

#endif /* ICP_7dof_TRANSFORM_ESTIMATION_H_ */
