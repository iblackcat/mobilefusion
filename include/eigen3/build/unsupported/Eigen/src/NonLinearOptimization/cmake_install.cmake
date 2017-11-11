# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen/src/NonLinearOptimization" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/chkder.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/covar.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/dogleg.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/fdjac1.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/HybridNonLinearSolver.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/LevenbergMarquardt.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/lmpar.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/qrsolv.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/r1mpyq.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/r1updt.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/src/NonLinearOptimization/rwupdt.h"
    )
endif()

