# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Array"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Cholesky"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/CholmodSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Core"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Dense"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Eigen"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Eigen2Support"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Eigenvalues"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Geometry"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Householder"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/IterativeLinearSolvers"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Jacobi"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/LeastSquares"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/LU"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/MetisSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/OrderingMethods"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/PardisoSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/PaStiXSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/QR"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/QtAlignedMalloc"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/Sparse"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SparseCholesky"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SparseCore"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SparseLU"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SparseQR"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SPQRSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/StdDeque"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/StdList"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/StdVector"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SuperLUSupport"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/SVD"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/UmfPackSupport"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/build/Eigen/src/cmake_install.cmake")

endif()

