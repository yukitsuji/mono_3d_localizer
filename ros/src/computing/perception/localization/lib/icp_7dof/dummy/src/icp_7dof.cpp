#include "icp_7dof/icp_7dof.h"

template class pcl::IterativeClosestPoint7dof<pcl::PointXYZ, pcl::PointXYZ, double>;
template class pcl::IterativeClosestPoint7dof<pcl::PointXYZ, pcl::PointXYZ, float>;
//template class pcl::IterativeClosestPoint7dof<pcl::PointXYZI, pcl::PointXYZI, double>;
//template class pcl::IterativeClosestPoint7dof<pcl::PointXYZI, pcl::PointXYZI, float>;

template class pcl::IterativeClosestPoint7dofWithNormals<pcl::PointNormal, pcl::PointNormal, double>;
//template class pcl::IterativeClosestPoint7dofWithNormals<pcl::PointNormal, pcl::PointNormal, double>;
