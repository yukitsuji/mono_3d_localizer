#ifndef ICP_7DOF_H_
#define ICP_7DOF_H_

// PCL includes
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_registration.h>
#include <pcl/registration/registration.h>
//#include "registration_7dof.h"
#include "icp_7dof_transform.h"
#include "icp_7dof_transform_lm.h"
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/default_convergence_criteria.h>

namespace pcl
{
  /** \brief @b IterativeClosestPoint7dof provides a base implementation of the Iterative Closest Point algorithm.
    * The transformation is estimated based on Singular Value Decomposition (SVD).
    *
    * The algorithm has several termination criteria:
    *
    * <ol>
    * <li>Number of iterations has reached the maximum user imposed number of iterations (via \ref setMaximumIterations)</li>
    * <li>The epsilon (difference) between the previous transformation and the current estimated transformation is smaller than an user imposed value (via \ref setTransformationEpsilon)</li>
    * <li>The sum of Euclidean squared errors is smaller than a user defined threshold (via \ref setEuclideanFitnessEpsilon)</li>
    * </ol>
    *
    *
    * Usage example:
    * \code
    * IterativeClosestPoint7dof<PointXYZ, PointXYZ> icp;
    * // Set the input source and target
    * icp.setInputCloud (cloud_source);
    * icp.setInputTarget (cloud_target);
    *
    * // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    * icp.setMaxCorrespondenceDistance (0.05);
    * // Set the maximum number of iterations (criterion 1)
    * icp.setMaximumIterations (50);
    * // Set the transformation epsilon (criterion 2)
    * icp.setTransformationEpsilon (1e-8);
    * // Set the euclidean distance difference epsilon (criterion 3)
    * icp.setEuclideanFitnessEpsilon (1);
    *
    * // Perform the alignment
    * icp.align (cloud_source_registered);
    *
    * // Obtain the transformation that aligned cloud_source to cloud_source_registered
    * Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    * \endcode
    *
    * \author Radu B. Rusu, Michael Dixon
    * \ingroup registration
    */
  class IterativeClosestPoint7dof : public Registration<pcl::PointXYZ, pcl::PointXYZ, double>
  {
    public:
      typedef typename Registration<pcl::PointXYZ, pcl::PointXYZ, double>::PointCloudSource PointCloudSource;
      typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
      typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

      typedef typename Registration<pcl::PointXYZ, pcl::PointXYZ, double>::PointCloudTarget PointCloudTarget;
      typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
      typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

      typedef PointIndices::Ptr PointIndicesPtr;
      typedef PointIndices::ConstPtr PointIndicesConstPtr;

      typedef boost::shared_ptr<IterativeClosestPoint7dof> Ptr;
      typedef boost::shared_ptr<const IterativeClosestPoint7dof> ConstPtr;

      typedef typename pcl::registration::TransformationEstimation7dof TransformationEstimation;
      typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;
      typedef typename TransformationEstimation::ConstPtr TransformationEstimationConstPtr;

      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::reg_name_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::getClassName;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::input_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::indices_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::target_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::nr_iterations_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::max_iterations_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::previous_transformation_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::final_transformation_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::transformation_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::transformation_epsilon_;
      // using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::transformation_rotation_epsilon_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::converged_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::corr_dist_threshold_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::inlier_threshold_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::min_number_correspondences_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::update_visualizer_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::euclidean_fitness_epsilon_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::correspondences_;
      // using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::transformation_estimation_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::correspondence_estimation_;
      using Registration<pcl::PointXYZ, pcl::PointXYZ, double>::correspondence_rejectors_;

      typename pcl::registration::DefaultConvergenceCriteria<double>::Ptr convergence_criteria_;
      typedef typename Registration<pcl::PointXYZ, pcl::PointXYZ, double>::Matrix4 Matrix4;

      /** \brief Empty constructor. */
      IterativeClosestPoint7dof ()
        : x_idx_offset_ (0)
        , y_idx_offset_ (0)
        , z_idx_offset_ (0)
        , nx_idx_offset_ (0)
        , ny_idx_offset_ (0)
        , nz_idx_offset_ (0)
        , use_reciprocal_correspondence_ (false)
        , source_has_normals_ (false)
        , target_has_normals_ (false)
      {
        reg_name_ = "IterativeClosestPoint7dof";
        transformation_estimation_.reset (new pcl::registration::TransformationEstimation7dofLM());
        correspondence_estimation_.reset (new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, double>);
        convergence_criteria_.reset(new pcl::registration::DefaultConvergenceCriteria<double> (nr_iterations_, transformation_, *correspondences_));
      };

      /** \brief Empty destructor */
      virtual ~IterativeClosestPoint7dof () {}

      void
      setTransformationEstimation (const TransformationEstimationPtr &te) { transformation_estimation_ = te; }

      /** \brief Returns a pointer to the DefaultConvergenceCriteria used by the IterativeClosestPoint7dof class.
        * This allows to check the convergence state after the align() method as well as to configure
        * DefaultConvergenceCriteria's parameters not available through the ICP API before the align()
        * method is called. Please note that the align method sets max_iterations_,
        * euclidean_fitness_epsilon_ and transformation_epsilon_ and therefore overrides the default / set
        * values of the DefaultConvergenceCriteria instance.
        * \return Pointer to the IterativeClosestPoint7dof's DefaultConvergenceCriteria.
        */
      inline typename pcl::registration::DefaultConvergenceCriteria<double>::Ptr
      getConvergeCriteria ()
      {
        return convergence_criteria_;
      }

      /** \brief Provide a pointer to the input source
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud source
        */
      virtual void
      setInputSource (const PointCloudSourceConstPtr &cloud)
      {
        Registration<pcl::PointXYZ, pcl::PointXYZ, double>::setInputSource (cloud);
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields (*cloud, fields);
        source_has_normals_ = false;
        for (size_t i = 0; i < fields.size (); ++i)
        {
          if      (fields[i].name == "x") x_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "y") y_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "z") z_idx_offset_ = fields[i].offset;
          else if (fields[i].name == "normal_x")
          {
            source_has_normals_ = true;
            nx_idx_offset_ = fields[i].offset;
          }
          else if (fields[i].name == "normal_y")
          {
            source_has_normals_ = true;
            ny_idx_offset_ = fields[i].offset;
          }
          else if (fields[i].name == "normal_z")
          {
            source_has_normals_ = true;
            nz_idx_offset_ = fields[i].offset;
          }
        }
      }

      /** \brief Provide a pointer to the input target
        * (e.g., the point cloud that we want to align to the target)
        *
        * \param[in] cloud the input point cloud target
        */
      virtual void
      setInputTarget (const PointCloudTargetConstPtr &cloud)
      {
        Registration<pcl::PointXYZ, pcl::PointXYZ, double>::setInputTarget (cloud);
        std::vector<pcl::PCLPointField> fields;
        pcl::getFields (*cloud, fields);
        target_has_normals_ = false;
        for (size_t i = 0; i < fields.size (); ++i)
        {
          if (fields[i].name == "normal_x" || fields[i].name == "normal_y" || fields[i].name == "normal_z")
          {
            target_has_normals_ = true;
            break;
          }
        }
      }

      /** \brief Set whether to use reciprocal correspondence or not
        *
        * \param[in] use_reciprocal_correspondence whether to use reciprocal correspondence or not
        */
      inline void
      setUseReciprocalCorrespondences (bool use_reciprocal_correspondence)
      {
        use_reciprocal_correspondence_ = use_reciprocal_correspondence;
      }

      /** \brief Obtain whether reciprocal correspondence are used or not */
      inline bool
      getUseReciprocalCorrespondences () const
      {
        return (use_reciprocal_correspondence_);
      }

    protected:

      /** \brief Apply a rigid transform to a given dataset. Here we check whether whether
        * the dataset has surface normals in addition to XYZ, and rotate normals as well.
        * \param[in] input the input point cloud
        * \param[out] output the resultant output point cloud
        * \param[in] transform a 4x4 rigid transformation
        * \note Can be used with cloud_in equal to cloud_out
        */
      virtual void
      transformCloud (const PointCloudSource &input,
                      PointCloudSource &output,
                      const Matrix4 &transform);

      /** \brief Rigid transformation computation method  with initial guess.
        * \param output the transformed input point cloud dataset using the rigid transformation found
        * \param guess the initial guess of the transformation to compute
        */
      virtual void
      computeTransformation (PointCloudSource &output, const Matrix4 &guess);

      /** \brief Looks at the Estimators and Rejectors and determines whether their blob-setter methods need to be called */
      virtual void
      determineRequiredBlobData ();

      /** \brief XYZ fields offset. */
      size_t x_idx_offset_, y_idx_offset_, z_idx_offset_;

      /** \brief Normal fields offset. */
      size_t nx_idx_offset_, ny_idx_offset_, nz_idx_offset_;

      /** \brief The correspondence type used for correspondence estimation. */
      bool use_reciprocal_correspondence_;

      /** \brief Internal check whether source dataset has normals or not. */
      bool source_has_normals_;
      /** \brief Internal check whether target dataset has normals or not. */
      bool target_has_normals_;

      /** \brief Checks for whether estimators and rejectors need various data */
      bool need_source_blob_, need_target_blob_;

      TransformationEstimationPtr transformation_estimation_;

  };
}

#include "impl/icp_7dof.hpp"

#endif  //#ifndef ICP_7DOF_H_
