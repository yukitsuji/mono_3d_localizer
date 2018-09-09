cmake_minimum_required(VERSION 3.1)

project(g2o)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/)

find_package(SuiteSparse REQUIRED)
find_package(BLAS REQUIRED)
find_package(LAPACK REQUIRED)
find_package(OpenMP)

SET(g2o_C_FLAGS "-std=c++11")
SET(g2o_CXX_FLAGS "-std=c++11")

include(CPack)
# The library prefix
set(LIB_PREFIX g2o_)

set(g2o_C_FLAGS)
set(g2o_CXX_FLAGS)

# For building the CHOLMOD / CSPARSE solvers
find_package(Cholmod)
find_package(BLAS)
find_package(LAPACK)

# Set standard installation directories
set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
set(INCLUDES_INSTALL_DIR ${INCLUDES_DESTINATION}/g2o)

list(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

option (G2O_USE_CHOLMOD "Build g2o with CHOLMOD support" ON)
set(CHOLMOD_FOUND TRUE)


option (G2O_USE_CSPARSE "Build g2o with CSParse support" ON)

find_package(CSparse REQUIRED)
set(CSPARSE_FOUND TRUE)
set(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")

# Eigen library parallelise itself, though, presumably due to performance issues
# OPENMP is experimental. We experienced some slowdown with it
set(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
if(G2O_USE_OPENMP)
  find_package(OpenMP)
  if(OPENMP_FOUND)
    set (G2O_OPENMP 1)
    set(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    message(STATUS "Compiling with OpenMP support")
  endif(OPENMP_FOUND)
endif(G2O_USE_OPENMP)


# C++11 support
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# specifying compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

find_package(Eigen3 NO_MODULE)
if (TARGET Eigen3::Eigen)
  set(G2O_EIGEN3_EIGEN_TARGET Eigen3::Eigen)
else()
  find_package(Eigen3 REQUIRED)
  include_directories(${EIGEN3_INCLUDE_DIR})
endif ()

# Set up the top-level include directories
include_directories(${g2o_SOURCE_DIR} ${PROJECT_BINARY_DIR})
include_directories(${SUITESPARSE_INCLUDE_DIRS})
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})

#configure_file(config.h.in "${PROJECT_BINARY_DIR}/g2o/config.h")
install(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${INCLUDES_DESTINATION}/g2o)

# Include the subdirectories
add_subdirectory(EXTERNAL)
#add_subdirectory(g2o)

add_library(g2o STATIC
    g2o/core/base_binary_edge.h
    g2o/core/base_binary_edge.hpp
    g2o/core/base_edge.h
    g2o/core/base_multi_edge.h
    g2o/core/base_multi_edge.hpp
    g2o/core/base_unary_edge.h
    g2o/core/base_unary_edge.hpp
    g2o/core/base_vertex.h
    g2o/core/base_vertex.hpp
    g2o/core/batch_stats.cpp
    g2o/core/batch_stats.h
    g2o/core/block_solver.h
    g2o/core/block_solver.hpp
    g2o/core/cache.cpp
    g2o/core/cache.h
    g2o/core/creators.h
    g2o/core/dynamic_aligned_buffer.hpp
    g2o/core/eigen_types.h
    g2o/core/estimate_propagator.cpp
    g2o/core/estimate_propagator.h
    g2o/core/factory.cpp
    g2o/core/factory.h
    g2o/core/g2o_core_api.h
    g2o/core/hyper_dijkstra.cpp
    g2o/core/hyper_dijkstra.h
    g2o/core/hyper_graph.cpp
    g2o/core/hyper_graph.h
    g2o/core/hyper_graph_action.cpp
    g2o/core/hyper_graph_action.h
    g2o/core/jacobian_workspace.cpp
    g2o/core/jacobian_workspace.h
    g2o/core/linear_solver.h
    g2o/core/marginal_covariance_cholesky.cpp
    g2o/core/marginal_covariance_cholesky.h
    g2o/core/matrix_operations.h
    g2o/core/matrix_structure.cpp
    g2o/core/matrix_structure.h
    g2o/core/openmp_mutex.h
    g2o/core/optimizable_graph.cpp
    g2o/core/optimizable_graph.h
    g2o/core/optimization_algorithm.cpp
    g2o/core/optimization_algorithm.h
    g2o/core/optimization_algorithm_dogleg.cpp
    g2o/core/optimization_algorithm_dogleg.h
    g2o/core/optimization_algorithm_factory.cpp
    g2o/core/optimization_algorithm_factory.h
    g2o/core/optimization_algorithm_gauss_newton.cpp
    g2o/core/optimization_algorithm_gauss_newton.h
    g2o/core/optimization_algorithm_levenberg.cpp
    g2o/core/optimization_algorithm_levenberg.h
    g2o/core/optimization_algorithm_property.h
    g2o/core/optimization_algorithm_with_hessian.cpp
    g2o/core/optimization_algorithm_with_hessian.h
    g2o/core/ownership.h
    g2o/core/parameter.cpp
    g2o/core/parameter.h
    g2o/core/parameter_container.cpp
    g2o/core/parameter_container.h
    g2o/core/robust_kernel.cpp
    g2o/core/robust_kernel.h
    g2o/core/robust_kernel_factory.cpp
    g2o/core/robust_kernel_factory.h
    g2o/core/robust_kernel_impl.cpp
    g2o/core/robust_kernel_impl.h
    g2o/core/solver.cpp
    g2o/core/solver.h
    g2o/core/sparse_block_matrix.h
    g2o/core/sparse_block_matrix.hpp
    g2o/core/sparse_block_matrix_ccs.h
    g2o/core/sparse_block_matrix_diagonal.h
    g2o/core/sparse_optimizer.cpp
    g2o/core/sparse_optimizer.h
    g2o/core/sparse_optimizer_terminate_action.cpp
    g2o/core/sparse_optimizer_terminate_action.h

    g2o/types/data/data_queue.cpp
    g2o/types/data/data_queue.h
    g2o/types/data/g2o_types_data_api.h
    g2o/types/data/laser_parameters.cpp
    g2o/types/data/laser_parameters.h
    g2o/types/data/raw_laser.cpp
    g2o/types/data/raw_laser.h
    g2o/types/data/robot_data.cpp
    g2o/types/data/robot_data.h
    g2o/types/data/robot_laser.cpp
    g2o/types/data/robot_laser.h
    g2o/types/data/types_data.cpp
    g2o/types/data/types_data.h
    g2o/types/data/vertex_ellipse.cpp
    g2o/types/data/vertex_ellipse.h
    g2o/types/data/vertex_tag.cpp
    g2o/types/data/vertex_tag.h
    g2o/types/icp/g2o_types_icp_api.h
    g2o/types/icp/types_icp.cpp
    g2o/types/icp/types_icp.h
    g2o/types/sba/g2o_types_sba_api.h
    g2o/types/sba/sbacam.h
    g2o/types/sba/types_sba.cpp
    g2o/types/sba/types_sba.h
    g2o/types/sba/types_six_dof_expmap.cpp
    g2o/types/sba/types_six_dof_expmap.h
    g2o/types/sclam2d/edge_se2_odom_differential_calib.cpp
    g2o/types/sclam2d/edge_se2_odom_differential_calib.h
    g2o/types/sclam2d/edge_se2_sensor_calib.cpp
    g2o/types/sclam2d/edge_se2_sensor_calib.h
    g2o/types/sclam2d/g2o_types_sclam2d_api.h
    g2o/types/sclam2d/odometry_measurement.cpp
    g2o/types/sclam2d/odometry_measurement.h
    g2o/types/sclam2d/types_sclam2d.cpp
    g2o/types/sclam2d/types_sclam2d.h
    g2o/types/sclam2d/vertex_odom_differential_params.cpp
    g2o/types/sclam2d/vertex_odom_differential_params.h
    g2o/types/sim3/sim3.h
    g2o/types/sim3/types_seven_dof_expmap.cpp
    g2o/types/sim3/types_seven_dof_expmap.h
    g2o/types/slam2d/edge_pointxy.cpp
    g2o/types/slam2d/edge_pointxy.h
    g2o/types/slam2d/edge_se2.cpp
    g2o/types/slam2d/edge_se2.h
    g2o/types/slam2d/edge_se2_lotsofxy.cpp
    g2o/types/slam2d/edge_se2_lotsofxy.h
    g2o/types/slam2d/edge_se2_offset.cpp
    g2o/types/slam2d/edge_se2_offset.h
    g2o/types/slam2d/edge_se2_pointxy.cpp
    g2o/types/slam2d/edge_se2_pointxy.h
    g2o/types/slam2d/edge_se2_pointxy_bearing.cpp
    g2o/types/slam2d/edge_se2_pointxy_bearing.h
    g2o/types/slam2d/edge_se2_pointxy_calib.cpp
    g2o/types/slam2d/edge_se2_pointxy_calib.h
    g2o/types/slam2d/edge_se2_pointxy_offset.cpp
    g2o/types/slam2d/edge_se2_pointxy_offset.h
    g2o/types/slam2d/edge_se2_prior.cpp
    g2o/types/slam2d/edge_se2_prior.h
    g2o/types/slam2d/edge_se2_twopointsxy.cpp
    g2o/types/slam2d/edge_se2_twopointsxy.h
    g2o/types/slam2d/edge_se2_xyprior.cpp
    g2o/types/slam2d/edge_se2_xyprior.h
    g2o/types/slam2d/edge_xy_prior.cpp
    g2o/types/slam2d/edge_xy_prior.h
    g2o/types/slam2d/g2o_types_slam2d_api.h
    g2o/types/slam2d/parameter_se2_offset.cpp
    g2o/types/slam2d/parameter_se2_offset.h
    g2o/types/slam2d/se2.h
    g2o/types/slam2d/types_slam2d.cpp
    g2o/types/slam2d/types_slam2d.h
    g2o/types/slam2d/vertex_point_xy.cpp
    g2o/types/slam2d/vertex_point_xy.h
    g2o/types/slam2d/vertex_se2.cpp
    g2o/types/slam2d/vertex_se2.h
    g2o/types/slam2d_addons/edge_line2d.cpp
    g2o/types/slam2d_addons/edge_line2d.h
    g2o/types/slam2d_addons/edge_line2d_pointxy.cpp
    g2o/types/slam2d_addons/edge_line2d_pointxy.h
    g2o/types/slam2d_addons/edge_se2_line2d.cpp
    g2o/types/slam2d_addons/edge_se2_line2d.h
    g2o/types/slam2d_addons/edge_se2_segment2d.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d.h
    g2o/types/slam2d_addons/edge_se2_segment2d_line.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d_line.h
    g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.cpp
    g2o/types/slam2d_addons/edge_se2_segment2d_pointLine.h
    g2o/types/slam2d_addons/g2o_types_slam2d_addons_api.h
    g2o/types/slam2d_addons/line_2d.h
    g2o/types/slam2d_addons/types_slam2d_addons.cpp
    g2o/types/slam2d_addons/types_slam2d_addons.h
    g2o/types/slam2d_addons/vertex_line2d.cpp
    g2o/types/slam2d_addons/vertex_line2d.h
    g2o/types/slam2d_addons/vertex_segment2d.cpp
    g2o/types/slam2d_addons/vertex_segment2d.h
    g2o/types/slam3d/dquat2mat.cpp
    g2o/types/slam3d/dquat2mat.h
    g2o/types/slam3d/dquat2mat.wxm
    g2o/types/slam3d/edge_pointxyz.cpp
    g2o/types/slam3d/edge_pointxyz.h
    g2o/types/slam3d/edge_se3.cpp
    g2o/types/slam3d/edge_se3.h
    g2o/types/slam3d/edge_se3_lotsofxyz.cpp
    g2o/types/slam3d/edge_se3_lotsofxyz.h
    g2o/types/slam3d/edge_se3_offset.cpp
    g2o/types/slam3d/edge_se3_offset.h
    g2o/types/slam3d/edge_se3_pointxyz.cpp
    g2o/types/slam3d/edge_se3_pointxyz.h
    g2o/types/slam3d/edge_se3_pointxyz_depth.cpp
    g2o/types/slam3d/edge_se3_pointxyz_depth.h
    g2o/types/slam3d/edge_se3_pointxyz_disparity.cpp
    g2o/types/slam3d/edge_se3_pointxyz_disparity.h
    g2o/types/slam3d/edge_se3_prior.cpp
    g2o/types/slam3d/edge_se3_prior.h
    g2o/types/slam3d/edge_xyz_prior.cpp
    g2o/types/slam3d/edge_xyz_prior.h
    g2o/types/slam3d/g2o_types_slam3d_api.h
    g2o/types/slam3d/isometry3d_gradients.cpp
    g2o/types/slam3d/isometry3d_gradients.h
    g2o/types/slam3d/isometry3d_mappings.cpp
    g2o/types/slam3d/isometry3d_mappings.h
    g2o/types/slam3d/parameter_camera.cpp
    g2o/types/slam3d/parameter_camera.h
    g2o/types/slam3d/parameter_se3_offset.cpp
    g2o/types/slam3d/parameter_se3_offset.h
    g2o/types/slam3d/parameter_stereo_camera.cpp
    g2o/types/slam3d/parameter_stereo_camera.h
    g2o/types/slam3d/se3_ops.h
    g2o/types/slam3d/se3_ops.hpp
    g2o/types/slam3d/se3quat.h
    g2o/types/slam3d/types_slam3d.cpp
    g2o/types/slam3d/types_slam3d.h
    g2o/types/slam3d/vertex_pointxyz.cpp
    g2o/types/slam3d/vertex_pointxyz.h
    g2o/types/slam3d/vertex_se3.cpp
    g2o/types/slam3d/vertex_se3.h
    g2o/types/slam3d_addons/edge_plane.cpp
    g2o/types/slam3d_addons/edge_plane.h
    g2o/types/slam3d_addons/edge_se3_calib.cpp
    g2o/types/slam3d_addons/edge_se3_calib.h
    g2o/types/slam3d_addons/edge_se3_euler.cpp
    g2o/types/slam3d_addons/edge_se3_euler.h
    g2o/types/slam3d_addons/edge_se3_line.cpp
    g2o/types/slam3d_addons/edge_se3_line.h
    g2o/types/slam3d_addons/edge_se3_plane_calib.cpp
    g2o/types/slam3d_addons/edge_se3_plane_calib.h
    g2o/types/slam3d_addons/g2o_types_slam3d_addons_api.h
    g2o/types/slam3d_addons/line3d.cpp
    g2o/types/slam3d_addons/line3d.h
    g2o/types/slam3d_addons/plane3d.h
    g2o/types/slam3d_addons/types_slam3d_addons.cpp
    g2o/types/slam3d_addons/types_slam3d_addons.h
    g2o/types/slam3d_addons/vertex_line3d.cpp
    g2o/types/slam3d_addons/vertex_line3d.h
    g2o/types/slam3d_addons/vertex_plane.cpp
    g2o/types/slam3d_addons/vertex_plane.h
    g2o/types/slam3d_addons/vertex_se3_euler.cpp
    g2o/types/slam3d_addons/vertex_se3_euler.h

    g2o/types/deprecated/slam3d/edge_se3_offset.cpp
    g2o/types/deprecated/slam3d/edge_se3_offset.h
    g2o/types/deprecated/slam3d/edge_se3_pointxyz.cpp
    g2o/types/deprecated/slam3d/edge_se3_pointxyz.h
    g2o/types/deprecated/slam3d/edge_se3_pointxyz_depth.cpp
    g2o/types/deprecated/slam3d/edge_se3_pointxyz_depth.h
    g2o/types/deprecated/slam3d/edge_se3_pointxyz_disparity.cpp
    g2o/types/deprecated/slam3d/edge_se3_pointxyz_disparity.h
    g2o/types/deprecated/slam3d/edge_se3_prior.cpp
    g2o/types/deprecated/slam3d/edge_se3_prior.h
    g2o/types/deprecated/slam3d/edge_se3_quat.cpp
    g2o/types/deprecated/slam3d/edge_se3_quat.h
    g2o/types/deprecated/slam3d/g2o_deprecated_types_slam3d_api.h
    g2o/types/deprecated/slam3d/parameter_camera.cpp
    g2o/types/deprecated/slam3d/parameter_camera.h
    g2o/types/deprecated/slam3d/parameter_se3_offset.cpp
    g2o/types/deprecated/slam3d/parameter_se3_offset.h
    g2o/types/deprecated/slam3d/se3quat_gradients.cpp
    g2o/types/deprecated/slam3d/se3quat_gradients.h
    g2o/types/deprecated/slam3d/types_slam3d.cpp
    g2o/types/deprecated/slam3d/types_slam3d.h
    g2o/types/deprecated/slam3d/vertex_pointxyz.cpp
    g2o/types/deprecated/slam3d/vertex_pointxyz.h
    g2o/types/deprecated/slam3d/vertex_se3_quat.cpp
    g2o/types/deprecated/slam3d/vertex_se3_quat.h

    g2o/stuff/color_macros.h
    g2o/stuff/command_args.cpp
    g2o/stuff/command_args.h
    g2o/stuff/filesys_tools.cpp
    g2o/stuff/filesys_tools.h
    g2o/stuff/g2o_stuff_api.h
    g2o/stuff/macros.h
    g2o/stuff/misc.h
    g2o/stuff/os_specific.c
    g2o/stuff/os_specific.h
    g2o/stuff/property.cpp
    g2o/stuff/property.h
    g2o/stuff/sampler.cpp
    g2o/stuff/sampler.h
    g2o/stuff/scoped_pointer.h
    g2o/stuff/sparse_helper.cpp
    g2o/stuff/sparse_helper.h
    g2o/stuff/string_tools.cpp
    g2o/stuff/string_tools.h
    g2o/stuff/tictoc.cpp
    g2o/stuff/tictoc.h
    g2o/stuff/timeutil.cpp
    g2o/stuff/timeutil.h
    g2o/stuff/unscented.h

    g2o/solvers/cholmod/linear_solver_cholmod.h
    g2o/solvers/cholmod/solver_cholmod.cpp
    g2o/solvers/csparse/csparse_extension.cpp
    g2o/solvers/csparse/csparse_extension.h
    g2o/solvers/csparse/csparse_helper.cpp
    g2o/solvers/csparse/csparse_helper.h
    g2o/solvers/csparse/g2o_csparse_api.h
    g2o/solvers/csparse/g2o_csparse_extension_api.h
    g2o/solvers/csparse/linear_solver_csparse.h
    g2o/solvers/csparse/solver_csparse.cpp
    g2o/solvers/dense/linear_solver_dense.h
    g2o/solvers/dense/solver_dense.cpp
    g2o/solvers/eigen/linear_solver_eigen.h
    g2o/solvers/eigen/solver_eigen.cpp
    g2o/solvers/pcg/linear_solver_pcg.h
    g2o/solvers/pcg/linear_solver_pcg.hpp
    g2o/solvers/pcg/solver_pcg.cpp
    g2o/solvers/slam2d_linear/g2o_slam2d_linear_api.h
    g2o/solvers/slam2d_linear/slam2d_linear.cpp
    g2o/solvers/slam2d_linear/solver_slam2d_linear.cpp
    g2o/solvers/slam2d_linear/solver_slam2d_linear.h
    g2o/solvers/structure_only/structure_only.cpp
    g2o/solvers/structure_only/structure_only_solver.h
)

target_link_libraries(g2o
#    PUBLIC ${SUITESPARSE_INCLUDE_DIRS}
    PUBLIC ${CHOLMOD_LIBRARIES}
    PUBLIC csparse
    PUBLIC ${G2O_EIGEN3_EIGEN_TARGET})
