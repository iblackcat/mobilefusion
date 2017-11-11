# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/LU" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU/Determinant.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU/FullPivLU.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU/Inverse.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU/PartialPivLU.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/LU/PartialPivLU_MKL.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/build/Eigen/src/LU/arch/cmake_install.cmake")

endif()

