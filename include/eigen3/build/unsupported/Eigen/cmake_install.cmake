# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/AdolcForward"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/AlignedVector3"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/ArpackSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/AutoDiff"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/BVH"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/FFT"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/IterativeSolvers"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/KroneckerProduct"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/LevenbergMarquardt"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/MatrixFunctions"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/MoreVectorization"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/MPRealSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/NonLinearOptimization"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/NumericalDiff"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/OpenGLSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/Polynomials"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/Skyline"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/SparseExtra"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/unsupported/Eigen/Splines"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/build/unsupported/Eigen/src/cmake_install.cmake")

endif()

