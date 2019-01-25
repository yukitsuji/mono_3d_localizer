#ifndef PCL_WARP_POINT_NONRIGID_7D_H_
#define PCL_WARP_POINT_NONRIGID_7D_H_

#include <boost/shared_ptr.hpp>
#include <pcl/point_types.h>
#include <pcl/registration/warp_point_rigid.h>

namespace pcl
{
  namespace registration
  {
    /** \brief @b WarpPointRigid3D enables 7D (3D rotation + 3D translation + 1D scale)
      * transformations for points.
      *
      * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
      * \author Yuki Tsuji
      * \ingroup registration
      */
    class WarpPointRigid7D : public WarpPointRigid<pcl::PointXYZ, pcl::PointXYZ, double>
    {
      public:
        using WarpPointRigid<pcl::PointXYZ, pcl::PointXYZ, double>::transform_matrix_;

        typedef typename WarpPointRigid<pcl::PointXYZ, pcl::PointXYZ, double>::Matrix4 Matrix4;
        typedef typename WarpPointRigid<pcl::PointXYZ, pcl::PointXYZ, double>::VectorX VectorX;

        typedef boost::shared_ptr<WarpPointRigid7D> Ptr;
        typedef boost::shared_ptr<const WarpPointRigid7D> ConstPtr;

        WarpPointRigid7D () : WarpPointRigid<pcl::PointXYZ, pcl::PointXYZ, double> (7) {}

        /** \brief Empty destructor */
        virtual ~WarpPointRigid7D () {}

        /** \brief Set warp parameters.
          * \note Assumes the quaternion parameters are normalized.
          * \param[in] p warp parameters (tx ty tz qx qy qz)
          */
        virtual void
        setParam (const VectorX& p)
        {
          assert (p.rows () == this->getDimension ());

          // Copy the rotation and translation components
          transform_matrix_.setZero ();
          transform_matrix_ (0, 3) = p[0];
          transform_matrix_ (1, 3) = p[1];
          transform_matrix_ (2, 3) = p[2];
          transform_matrix_ (3, 3) = 1;

          // Compute w from the unit quaternion
          Eigen::Quaternion<double> q (0, p[3], p[4], p[5]);
          q.w () = static_cast<double> (sqrt (1 - q.dot (q)));
          q.normalize ();
          // double scale = 1;
          double scale = p[6] ? p[6] : 1;
          transform_matrix_.topLeftCorner (3, 3) = q.toRotationMatrix () * scale;
        }
    };
  }
}

//template class pcl::registration::WarpPointRigid7D<pcl::PointXYZI, pcl::PointXYZI, double>;
//template class pcl::registration::WarpPointRigid7D<pcl::PointXYZI, pcl::PointXYZI, float>;
// template class pcl::registration::WarpPointRigid7D<pcl::PointXYZ, pcl::PointXYZ, double>;
// template class pcl::registration::WarpPointRigid7D<pcl::PointXYZ, pcl::PointXYZ, float>;
#endif
