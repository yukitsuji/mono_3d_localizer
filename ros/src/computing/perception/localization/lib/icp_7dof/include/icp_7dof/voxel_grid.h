#ifndef ICP_7DOF_VOXEL_GRID_H_
#define ICP_7DOF_VOXEL_GRID_H_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <float.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace icp_7dof {

class VoxelGrid {
public:
    VoxelGrid();

    /* Set input points */
    void setInput(pcl::PointCloud<pcl::PointXYZ>::Ptr input);

    /* For each input point, search for voxels whose distance between their centroids and
     * the input point are less than radius.
     * The output is a list of candidate voxel ids */
    void radiusSearch(pcl::PointXYZ query_point, float radius, std::vector<int> &voxel_ids, int max_nn = INT_MAX);
    pcl::PointCloud<pcl::PointXYZ>::Ptr setPointsRaw(pcl::PointCloud<pcl::PointXYZ>::Ptr priorMap);
    int getVoxelNum() const;

	float getMaxX() const;
	float getMaxY() const;
	float getMaxZ() const;

	float getMinX() const;
	float getMinY() const;
	float getMinZ() const;

	float getVoxelX() const;
	float getVoxelY() const;
	float getVoxelZ() const;

	int getMaxBX() const;
	int getMaxBY() const;
	int getMaxBZ() const;

	int getMinBX() const;
	int getMinBY() const;
	int getMinBZ() const;

	int getVgridX() const;
	int getVgridY() const;
	int getVgridZ() const;

	void setLeafSize(float voxel_x, float voxel_y, float voxel_z);

  inline double getLeafSize()
  {
      return resolution_x_;
  }

	/* Searching for the nearest point of each input query point.
	 * Return the distance between the query point and its nearest neighbor.
	 * If the distance is larger than max_range, then return DBL_MAX. */

	double nearestNeighborDistance(pcl::PointXYZ query_point, float max_range);

  void setMinimumPoints(int min_points)
  {
    min_points_per_voxel_ = min_points;
  }

	Eigen::Vector3d getCentroid(int voxel_id) const;
	Eigen::Matrix3d getCovariance(int voxel_id) const;
	Eigen::Matrix3d getInverseCovariance(int voxel_id) const;

	void update(pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud);

  bool searchFittedVoxel(pcl::PointXYZ p);

private:

	typedef struct {
		int x, y, z;
	} OctreeDim;

	/* Construct the voxel grid and the build the octree. */
	void initialize();

	/* Put points into voxels */
	void scatterPointsToVoxelGrid();

	/* Compute centroids and covariances of voxels. */
	void computeCentroidAndCovariance();

	/* Find boundaries of input point cloud and compute
	 * the number of necessary voxels as well as boundaries
	 * measured in number of leaf size */
	void findBoundaries();

	void findBoundaries(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
							float &max_x, float &max_y, float &max_z,
							float &min_x, float &min_y, float &min_z);

	int voxelId(pcl::PointXYZ p);

	int voxelId(pcl::PointXYZ p,
				float voxel_x, float voxel_y, float voxel_z,
				int min_b_x, int min_b_y, int min_b_z,
				int vgrid_x, int vgrid_y, int vgrid_z);

	int voxelId(int idx, int idy, int idz,
				int min_b_x, int min_b_y, int min_b_z,
				int size_x, int size_y, int size_z);

	/* Private methods for merging new point cloud to the current point cloud */
	void updateBoundaries(float max_x, float max_y, float max_z,
							float min_x, float min_y, float min_z);

	void updateVoxelContent(typename pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud);

	int nearestVoxel(pcl::PointXYZ query_point, Eigen::Matrix<float, 6, 1> boundaries, float max_range);

	int roundUp(int input, int factor);

	int roundDown(int input, int factor);

	int div(int input, int divisor);

	//Coordinate of input points
	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;

	int voxel_num_;						// Number of voxels
	float max_x_, max_y_, max_z_;		// Upper bounds of the grid (maximum coordinate)
	float min_x_, min_y_, min_z_;		// Lower bounds of the grid (minimum coordinate)
	float resolution_x_, resolution_y_, resolution_z_;	// Leaf size, a.k.a, size of each voxel

	int max_b_x_, max_b_y_, max_b_z_;	// Upper bounds of the grid, measured in number of voxels
	int min_b_x_, min_b_y_, min_b_z_;	// Lower bounds of the grid, measured in number of voxels
	int vgrid_x_, vgrid_y_, vgrid_z_;	// Size of the voxel grid, measured in number of voxels
	int min_points_per_voxel_;			// Minimum number of points per voxel. If the number of points
										// per voxel is less than this number, then the voxel is ignored
										// during computation (treated like it contains no point)

	boost::shared_ptr<std::vector<Eigen::Vector3d> > centroid_; // 3x1 Centroid vectors of voxels
	boost::shared_ptr<std::vector<Eigen::Matrix3d> > icovariance_; // Inverse covariance matrixes of voxel
	boost::shared_ptr<std::vector<std::vector<int> > > points_id_; // Indexes of points belong to each voxel
  boost::shared_ptr<std::vector<int> > points_per_voxel_; // Number of points belong to each voxel
  boost::shared_ptr<std::vector<Eigen::Matrix<double,3,1>>> ievals_; // Number of points belong to each voxel
  boost::shared_ptr<std::vector<Eigen::Matrix3d >> ievecs_inv_; // Number of points belong to each voxel
  boost::shared_ptr<std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > > points_raw_per_voxel_; // (may differ from size of each vector in points_id_
        // because of changes made during computing covariances
	boost::shared_ptr<std::vector<Eigen::Vector3d> > tmp_centroid_;
	boost::shared_ptr<std::vector<Eigen::Matrix3d> > tmp_cov_;

	int real_max_bx_, real_max_by_, real_max_bz_;
	int real_min_bx_, real_min_by_, real_min_bz_;

	static const int MAX_BX_ = 16;
	static const int MAX_BY_ = 16;
	static const int MAX_BZ_ = 8;
};
}

#endif
