# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/Eigen")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/SparseLU" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLUImpl.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_column_bmod.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_column_dfs.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_copy_to_ucol.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_gemm_kernel.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_heap_relax_snode.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_kernel_bmod.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_Memory.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_panel_bmod.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_panel_dfs.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_pivotL.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_pruneL.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_relax_snode.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_Structs.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_SupernodalMatrix.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseLU/SparseLU_Utils.h"
    )
endif()

