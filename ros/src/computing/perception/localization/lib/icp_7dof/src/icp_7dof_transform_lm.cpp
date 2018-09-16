#include "icp_7dof/icp_7dof_transform_lm.h"
template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZI, pcl::PointXYZI, double>;
template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZI, pcl::PointXYZI, float>;
template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZ, pcl::PointXYZ, double>;
template class pcl::registration::TransformationEstimation7dofLM<pcl::PointXYZ, pcl::PointXYZ, float>;
