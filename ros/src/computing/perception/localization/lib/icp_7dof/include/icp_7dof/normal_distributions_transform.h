#ifndef CPU_NDT_H_
#define CPU_NDT_H_

#include "registration.h"
#include "voxel_grid.h"
#include <eigen3/Eigen/Geometry>

namespace icp_7dof {

class NormalDistributionsTransform: public Registration {
public:
	NormalDistributionsTransform();

	NormalDistributionsTransform(const NormalDistributionsTransform &other);

	void setStepSize(double step_size);

	void setResolution(float resolution);

	void setOutlierRatio(double olr);

	void setVoxelGrid(icp_7dof::VoxelGrid &voxel_grid);

	double getStepSize() const;

	float getResolution() const;

	double getOutlierRatio() const;

	double getTransformationProbability() const;

	int getRealIterations();

	/* Set the input map points */
	void setInputTarget(typename pcl::PointCloud<pcl::PointXYZ>::Ptr input);

	/* Compute and get fitness score */
	double getFitnessScore(double max_range = DBL_MAX);

	void updateVoxelGrid(typename pcl::PointCloud<pcl::PointXYZ>::Ptr new_cloud);

	icp_7dof::VoxelGrid voxel_grid_;

protected:
	void computeTransformation(const Eigen::Matrix<float, 4, 4> &guess);


	using Registration::transformation_epsilon_;
	using Registration::max_iterations_;
	using Registration::source_cloud_;
	using Registration::trans_cloud_;
	using Registration::converged_;
	using Registration::nr_iterations_;
	using Registration::final_transformation_;
	using Registration::transformation_;
	using Registration::previous_transformation_;
	using Registration::target_cloud_updated_;
	using Registration::target_cloud_;

private:
	//Copied from ndt.h
    double auxilaryFunction_PsiMT (double a, double f_a, double f_0, double g_0, double mu = 1.e-4);

    //Copied from ndt.h
    double auxilaryFunction_dPsiMT (double g_a, double g_0, double mu = 1.e-4);

    double updateIntervalMT (double &a_l, double &f_l, double &g_l,
								double &a_u, double &f_u, double &g_u,
								double a_t, double f_t, double g_t);

    double trialValueSelectionMT (double a_l, double f_l, double g_l,
									double a_u, double f_u, double g_u,
									double a_t, double f_t, double g_t);

	void computeAngleDerivatives(Eigen::Matrix<double, 7, 1> pose, bool compute_hessian = true);

	double computeStepLengthMT(const Eigen::Matrix<double, 7, 1> &x, Eigen::Matrix<double, 7, 1> &step_dir,
								double step_init, double step_max, double step_min, double &score,
								Eigen::Matrix<double, 7, 1> &score_gradient, Eigen::Matrix<double, 7, 7> &hessian,
								typename pcl::PointCloud<pcl::PointXYZ> &trans_cloud);

	void computeHessian(Eigen::Matrix<double, 7, 7> &hessian, typename pcl::PointCloud<pcl::PointXYZ> &trans_cloud, Eigen::Matrix<double, 7, 1> &p);

	double computeDerivatives(Eigen::Matrix<double, 7, 1> &score_gradient, Eigen::Matrix<double, 7, 7> &hessian,
								typename pcl::PointCloud<pcl::PointXYZ> &trans_cloud,
								Eigen::Matrix<double, 7, 1> pose, bool compute_hessian = true);
	void computePointDerivatives(Eigen::Vector3d &x, Eigen::Matrix<double, 3, 7> &point_gradient, Eigen::Matrix<double, 21, 7> &point_hessian, bool computeHessian = true);
	double updateDerivatives(Eigen::Matrix<double, 7, 1> &score_gradient, Eigen::Matrix<double, 7, 7> &hessian,
								Eigen::Matrix<double, 3, 7> point_gradient, Eigen::Matrix<double, 21, 7> point_hessian,
								Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv, bool compute_hessian = true);
	void updateHessian(Eigen::Matrix<double, 7, 7> &hessian,
						Eigen::Matrix<double, 3, 7> point_gradient, Eigen::Matrix<double, 21, 7> point_hessian,
						Eigen::Vector3d &x_trans, Eigen::Matrix3d &c_inv);

	double gauss_d1_, gauss_d2_;
	double outlier_ratio_;
	Eigen::Vector3d j_ang_a_, j_ang_b_, j_ang_c_, j_ang_d_, j_ang_e_, j_ang_f_, j_ang_g_, j_ang_h_;
	Eigen::Vector3d j_ang_i_, j_ang_j_, j_ang_k_;

	Eigen::Vector3d h_ang_a2_, h_ang_a3_,
					h_ang_b2_, h_ang_b3_,
					h_ang_c2_, h_ang_c3_,
					h_ang_d1_, h_ang_d2_, h_ang_d3_,
					h_ang_e1_, h_ang_e2_, h_ang_e3_,
					h_ang_f1_, h_ang_f2_, h_ang_f3_;

	Eigen::Vector3d h_ang_g1_, h_ang_g2_, h_ang_g3_,
	                h_ang_h1_, h_ang_h2_, h_ang_h3_,
	                h_ang_i1_, h_ang_i2_, h_ang_i3_;

	double step_size_;
	float resolution_;
	double trans_probability_;

	int real_iterations_;
};
}
#endif
