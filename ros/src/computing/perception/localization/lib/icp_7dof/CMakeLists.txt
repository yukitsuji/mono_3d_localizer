cmake_minimum_required(VERSION 2.8.3)
project(icp_7dof)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)

find_package(Eigen3 QUIET)
find_package(Boost)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES icp_7dof                   #The exported libraries from the project
        DEPENDS PCL
)

include_directories(
        ${PCL_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
)

set(srcs
        src/icp_7dof.cpp
        src/icp_7dof_transform_lm.cpp
        )

set(incs
        include/icp_7dof/icp_7dof.h
        include/icp_7dof/icp_7dof_transform.h
        include/icp_7dof/warp_point_nonrigid_7d.h
        include/icp_7dof/icp_7dof_transform_lm.h
        )

add_library(icp_7dof ${incs} ${srcs})

target_link_libraries(icp_7dof
        ${PCL_LIBRARIES}
        ${catkin_LIBRARIES}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        )


install(TARGETS icp_7dof
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
