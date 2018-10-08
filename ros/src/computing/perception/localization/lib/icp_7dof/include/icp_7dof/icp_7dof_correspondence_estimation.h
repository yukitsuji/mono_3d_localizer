#ifndef ICP_7DOF_CORRESPONDENCE_ESTIMATION_H_
#define ICP_7DOF_CORRESPONDENCE_ESTIMATION_H_

#include <string>

#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/pcl_macros.h>

#include <pcl/registration/correspondence_types.h>

namespace pcl
{
  namespace registration
  {
    /** \brief Abstract @b ICPCorrespondenceEstimationBase class. 
      * All correspondence estimation methods should inherit from this.
      * \author Radu B. Rusu
      * \ingroup registration
      */
    class ICPCorrespondenceEstimationBase: public PCLBase<pcl::PointXYZ>
    {
      public:
        typedef boost::shared_ptr<ICPCorrespondenceEstimationBase > Ptr;
        typedef boost::shared_ptr<const ICPCorrespondenceEstimationBase > ConstPtr;

        // using PCLBase<pcl::PointXYZ>::initCompute;
        using PCLBase<pcl::PointXYZ>::deinitCompute;
        using PCLBase<pcl::PointXYZ>::input_;
        using PCLBase<pcl::PointXYZ>::indices_;
        using PCLBase<pcl::PointXYZ>::setIndices;

        typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
        typedef typename KdTree::Ptr KdTreePtr;

        typedef pcl::search::KdTree<pcl::PointXYZ> KdTreeReciprocal;
        typedef typename KdTree::Ptr KdTreeReciprocalPtr;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        ICPCorrespondenceEstimationBase () 
          : corr_name_ ("ICPCorrespondenceEstimationBase")
          , tree_ (new pcl::search::KdTree<pcl::PointXYZ>)
          , tree_reciprocal_ (new pcl::search::KdTree<pcl::PointXYZ>)
          , target_ ()
          , target_indices_ ()
          , point_representation_ ()
          , input_transformed_ ()
          , input_fields_ ()
          , target_cloud_updated_ (true)
          , source_cloud_updated_ (true)
          , force_no_recompute_ (false)
          , force_no_recompute_reciprocal_ (false)
        {
        }
      
        /** \brief Empty destructor */
        virtual ~ICPCorrespondenceEstimationBase () {}

        /** \brief Provide a pointer to the input source 
          * (e.g., the point cloud that we want to align to the target)
          *
          * \param[in] cloud the input point cloud source
          */
        inline void 
        setInputSource (const PointCloudSourceConstPtr &cloud)
        {
          source_cloud_updated_ = true;
          PCLBase<pcl::PointXYZ>::setInputCloud (cloud);
          pcl::getFields (*cloud, input_fields_);
        }

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudSourceConstPtr const 
        getInputSource () 
        { 
          return (input_ ); 
        }

        /** \brief Provide a pointer to the input target 
          * (e.g., the point cloud that we want to align the input source to)
          * \param[in] cloud the input point cloud target
          */
        virtual void 
        setInputTarget (const PointCloudTargetConstPtr &cloud);

        /** \brief Get a pointer to the input point cloud dataset target. */
        inline PointCloudTargetConstPtr const 
        getInputTarget () { return (target_ ); }


        /** \brief See if this rejector requires source normals */
        virtual bool
        requiresSourceNormals () const
        { return (false); }

        /** \brief Abstract method for setting the source normals */
        virtual void
        setSourceNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setSourceNormals] This class does not require input source normals", getClassName ().c_str ());
        }
        
        /** \brief See if this rejector requires target normals */
        virtual bool
        requiresTargetNormals () const
        { return (false); }

        /** \brief Abstract method for setting the target normals */
        virtual void
        setTargetNormals (pcl::PCLPointCloud2::ConstPtr /*cloud2*/)
        {
          PCL_WARN ("[pcl::registration::%s::setTargetNormals] This class does not require input target normals", getClassName ().c_str ());
        }

        /** \brief Provide a pointer to the vector of indices that represent the 
          * input source point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesSource (const IndicesPtr &indices)
        {
          setIndices (indices);
        }

        /** \brief Get a pointer to the vector of indices used for the source dataset. */
        inline IndicesPtr const 
        getIndicesSource () { return (indices_); }

        /** \brief Provide a pointer to the vector of indices that represent the input target point cloud.
          * \param[in] indices a pointer to the vector of indices 
          */
        inline void
        setIndicesTarget (const IndicesPtr &indices)
        {
          target_cloud_updated_ = true;
          target_indices_ = indices;
        }

        /** \brief Get a pointer to the vector of indices used for the target dataset. */
        inline IndicesPtr const 
        getIndicesTarget () { return (target_indices_); }

        /** \brief Provide a pointer to the search object used to find correspondences in
          * the target cloud.
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputTarget. Only use if you are 
          * confident that the tree will be set correctly.
          */
        inline void
        setSearchMethodTarget (const KdTreePtr &tree, 
                               bool force_no_recompute = false) 
        { 
          tree_ = tree; 
          if (force_no_recompute)
          {
            force_no_recompute_ = true;
          }
          // Since we just set a new tree, we need to check for updates
          target_cloud_updated_ = true;
        }

        /** \brief Get a pointer to the search method used to find correspondences in the
          * target cloud. */
        inline KdTreePtr
        getSearchMethodTarget () const
        {
          return (tree_);
        }

        /** \brief Provide a pointer to the search object used to find correspondences in
          * the source cloud (usually used by reciprocal correspondence finding).
          * \param[in] tree a pointer to the spatial search object.
          * \param[in] force_no_recompute If set to true, this tree will NEVER be 
          * recomputed, regardless of calls to setInputSource. Only use if you are 
          * extremely confident that the tree will be set correctly.
          */
        inline void
        setSearchMethodSource (const KdTreeReciprocalPtr &tree, 
                               bool force_no_recompute = false) 
        { 
          tree_reciprocal_ = tree; 
          if ( force_no_recompute )
          {
            force_no_recompute_reciprocal_ = true;
          }
          // Since we just set a new tree, we need to check for updates
          source_cloud_updated_ = true;
        }

        /** \brief Get a pointer to the search method used to find correspondences in the
          * source cloud. */
        inline KdTreeReciprocalPtr
        getSearchMethodSource () const
        {
          return (tree_reciprocal_);
        }

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ()) = 0;

        /** \brief Provide a boost shared pointer to the PointRepresentation to be used 
          * when searching for nearest neighbors.
          *
          * \param[in] point_representation the PointRepresentation to be used by the 
          * k-D tree for nearest neighbor search
          */
        inline void
        setPointRepresentation (const PointRepresentationConstPtr &point_representation)
        {
          point_representation_ = point_representation;
        }

        /** \brief Clone and cast to ICPCorrespondenceEstimationBase */
        virtual boost::shared_ptr< ICPCorrespondenceEstimationBase > clone () const = 0;

      protected:
        /** \brief The correspondence estimation method name. */
        std::string corr_name_;

        /** \brief A pointer to the spatial search object used for the target dataset. */
        KdTreePtr tree_;

        /** \brief A pointer to the spatial search object used for the source dataset. */
        KdTreeReciprocalPtr tree_reciprocal_;


        
        /** \brief The input point cloud dataset target. */
        PointCloudTargetConstPtr target_;

        /** \brief The target point cloud dataset indices. */
        IndicesPtr target_indices_;

        /** \brief The point representation used (internal). */
        PointRepresentationConstPtr point_representation_;

        /** \brief The transformed input source point cloud dataset. */
        PointCloudTargetPtr input_transformed_;

        /** \brief The types of input point fields available. */
        std::vector<pcl::PCLPointField> input_fields_;

        /** \brief Abstract class get name method. */
        inline const std::string& 
        getClassName () const { return (corr_name_); }

        /** \brief Internal computation initialization. */
        bool
        initCompute ();
        
        /** \brief Internal computation initialization for reciprocal correspondences. */
        bool
        initComputeReciprocal ();

        /** \brief Variable that stores whether we have a new target cloud, meaning we need to pre-process it again.
         * This way, we avoid rebuilding the kd-tree for the target cloud every time the determineCorrespondences () method
         * is called. */
        bool target_cloud_updated_;
        /** \brief Variable that stores whether we have a new source cloud, meaning we need to pre-process it again.
         * This way, we avoid rebuilding the reciprocal kd-tree for the source cloud every time the determineCorrespondences () method
         * is called. */
        bool source_cloud_updated_;
        /** \brief A flag which, if set, means the tree operating on the target cloud 
         * will never be recomputed*/
        bool force_no_recompute_;
        
        /** \brief A flag which, if set, means the tree operating on the source cloud 
         * will never be recomputed*/
        bool force_no_recompute_reciprocal_;

     };

    /** \brief @b ICPCorrespondenceEstimation represents the base class for
      * determining correspondences between target and query point
      * sets/features.
      *
      * Code example:
      *
      * \code
      * pcl::PointCloud<pcl::PointXYZRGBA>::Ptr source, target;
      * // ... read or fill in source and target
      * pcl::ICPCorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ> est;
      * est.setInputSource (source);
      * est.setInputTarget (target);
      *
      * pcl::Correspondences all_correspondences;
      * // Determine all reciprocal correspondences
      * est.determineReciprocalCorrespondences (all_correspondences);
      * \endcode
      *
      * \author Radu B. Rusu, Michael Dixon, Dirk Holz
      * \ingroup registration
      */
    class ICPCorrespondenceEstimation : public ICPCorrespondenceEstimationBase
    {
      public:
        typedef boost::shared_ptr<ICPCorrespondenceEstimation > Ptr;
        typedef boost::shared_ptr<const ICPCorrespondenceEstimation > ConstPtr;

        using ICPCorrespondenceEstimationBase::point_representation_;
        using ICPCorrespondenceEstimationBase::input_transformed_;
        using ICPCorrespondenceEstimationBase::tree_;
        using ICPCorrespondenceEstimationBase::tree_reciprocal_;
        using ICPCorrespondenceEstimationBase::target_;
        using ICPCorrespondenceEstimationBase::corr_name_;
        using ICPCorrespondenceEstimationBase::target_indices_;
        using ICPCorrespondenceEstimationBase::getClassName;
        using ICPCorrespondenceEstimationBase::initCompute;
        using ICPCorrespondenceEstimationBase::initComputeReciprocal;
        using ICPCorrespondenceEstimationBase::input_;
        using ICPCorrespondenceEstimationBase::indices_;
        using ICPCorrespondenceEstimationBase::input_fields_;
        using PCLBase<pcl::PointXYZ>::deinitCompute;

        typedef pcl::search::KdTree<pcl::PointXYZ> KdTree;
        typedef typename pcl::search::KdTree<pcl::PointXYZ>::Ptr KdTreePtr;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;

        typedef pcl::PointCloud<pcl::PointXYZ> PointCloudTarget;
        typedef typename PointCloudTarget::Ptr PointCloudTargetPtr;
        typedef typename PointCloudTarget::ConstPtr PointCloudTargetConstPtr;

        typedef typename KdTree::PointRepresentationConstPtr PointRepresentationConstPtr;

        /** \brief Empty constructor. */
        ICPCorrespondenceEstimation () 
        {
          corr_name_  = "ICPCorrespondenceEstimation";
        }
      
        /** \brief Empty destructor */
        virtual ~ICPCorrespondenceEstimation () {}

        /** \brief Determine the correspondences between input and target cloud.
          * \param[out] correspondences the found correspondences (index of query point, index of target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineCorrespondences (pcl::Correspondences &correspondences,
                                  double max_distance = std::numeric_limits<double>::max ());

        /** \brief Determine the reciprocal correspondences between input and target cloud.
          * A correspondence is considered reciprocal if both Src_i has Tgt_i as a 
          * correspondence, and Tgt_i has Src_i as one.
          *
          * \param[out] correspondences the found correspondences (index of query and target point, distance)
          * \param[in] max_distance maximum allowed distance between correspondences
          */
        virtual void 
        determineReciprocalCorrespondences (pcl::Correspondences &correspondences,
                                            double max_distance = std::numeric_limits<double>::max ());

        
        /** \brief Clone and cast to ICPCorrespondenceEstimationBase */
        virtual boost::shared_ptr< ICPCorrespondenceEstimationBase > 
        clone () const
        {
          Ptr copy (new ICPCorrespondenceEstimation (*this));
          return (copy);
        }
     };
  }
}

#endif /* ICP_7DOF_CORRESPONDENCE_ESTIMATION_H_ */

