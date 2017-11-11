# Install script for directory: E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen/src/SparseCore" TYPE FILE FILES
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/AmbiVector.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/CompressedStorage.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/ConservativeSparseSparseProduct.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/MappedSparseMatrix.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseBlock.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseColEtree.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseCwiseBinaryOp.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseCwiseUnaryOp.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseDenseProduct.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseDiagonalProduct.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseDot.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseFuzzy.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseMatrix.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseMatrixBase.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparsePermutation.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseProduct.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseRedux.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseSelfAdjointView.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseSparseProductWithPruning.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseTranspose.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseTriangularView.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseUtil.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseVector.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/SparseView.h"
    "E:/code/c/mobilefusion/small_resolution/mobilefusion/include/eigen3/Eigen/src/SparseCore/TriangularSolver.h"
    )
endif()

