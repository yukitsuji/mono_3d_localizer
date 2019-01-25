#include "icp_7dof/registration.h"
#include <iostream>

namespace icp_7dof {

Registration::Registration()
{
	max_iterations_ = 0;

	converged_ = false;
	nr_iterations_ = 0;

	transformation_epsilon_ = 0;
	target_cloud_updated_ = true;

	trans_cloud_.points.clear();
}

Registration::~Registration()
{
	return;
}

void Registration::setTransformationEpsilon(double trans_eps)
{
	transformation_epsilon_ = trans_eps;
}

double Registration::getTransformationEpsilon() const
{
	return transformation_epsilon_;
}

void Registration::setMaximumIterations(int max_itr)
{
	max_iterations_ = max_itr;
}

int Registration::getMaximumIterations() const
{
	return max_iterations_;
}

Eigen::Matrix<float, 4, 4> Registration::getFinalTransformation() const
{
	return final_transformation_;
}

int Registration::getFinalNumIteration() const
{
	return nr_iterations_;
}

bool Registration::hasConverged() const
{
	return converged_;
}

void Registration::setInputSource(typename pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	source_cloud_ = input;
}


//Set input MAP data
void Registration::setInputTarget(typename pcl::PointCloud<pcl::PointXYZ>::Ptr input)
{
	target_cloud_ = input;
}

void Registration::align(const Eigen::Matrix<float, 4, 4> &guess)
{
	converged_ = false;

	final_transformation_ = transformation_ = previous_transformation_ = Eigen::Matrix<float, 4, 4>::Identity();

	trans_cloud_.points.resize(source_cloud_->points.size());

	for (int i = 0; i < trans_cloud_.points.size(); i++) {
		trans_cloud_.points[i] = source_cloud_->points[i];
	}

	computeTransformation(guess);
}

void Registration::align(typename pcl::PointCloud<pcl::PointXYZ> &output, const Eigen::Matrix<float, 4, 4> &guess)
{
	align(guess);
}

void Registration::computeTransformation(const Eigen::Matrix<float, 4, 4> &guess) {
	printf("Unsupported by Registration\n");
}
}
