CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(g2o_frontend)

# The library prefix
SET(LIB_PREFIX g2o_frontend)

SET(g2o_frontend_C_FLAGS)
SET(g2o_frontend_CXX_FLAGS)

# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# postfix, based on type
SET(CMAKE_DEBUG_POSTFIX "_d" CACHE STRING "postfix applied to debug build of libraries")
SET(CMAKE_RELEASE_POSTFIX "" CACHE STRING "postfix applied to release build of libraries")
SET(CMAKE_RELWITHDEBINFO_POSTFIX "_rd" CACHE STRING "postfix applied to release-with-debug-information libraries")
SET(CMAKE_MINSIZEREL_POSTFIX "_s" CACHE STRING "postfix applied to minimium-size-build libraries")

# work out the postfix; required where we use OUTPUT_NAME
IF(CMAKE_BUILD_TYPE MATCHES Release)
  SET(EXE_POSTFIX)
ELSEIF(CMAKE_BUILD_TYPE MATCHES Debug)
  SET(EXE_POSTFIX ${CMAKE_DEBUG_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
  SET(EXE_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
ELSEIF(CMAKE_BUILD_TYPE MATCHES MinSizeRel)
  SET(EXE_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
ENDIF(CMAKE_BUILD_TYPE MATCHES Release)

# Allow the developer to select if Dynamic or Static libraries are built
IF(MSVC) # MSVC only works if we link statically
  OPTION (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" OFF)
ELSE(MSVC)
  OPTION (BUILD_SHARED_LIBS "Build Shared Libraries (preferred and required for the g2o plugin system)" ON)
ENDIF()
SET (G2O_FRONTEND_LIB_TYPE STATIC)
IF (BUILD_SHARED_LIBS)
  SET (G2O_FRONTEND_LIB_TYPE SHARED)
ENDIF()

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# On the Mac platform, configure the RPATH as per the install, to
# avoid the problem of loading both the built and installed versions
# of the shared targets
IF(APPLE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
  SET(CMAKE_INSTALL_RPATH "")
ENDIF(APPLE)

# Set the output directory for the build executables and libraries
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_frontend_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${g2o_frontend_SOURCE_DIR}/bin)
IF(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_frontend_SOURCE_DIR}/bin)
ELSE(WIN32)
  SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_frontend_SOURCE_DIR}/lib)
ENDIF(WIN32)

# Set search directory for looking for our custom CMake scripts to
# look for SuiteSparse, QGLViewer, and Eigen3.
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${g2o_frontend_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(WIN32)
  IF(CYGWIN)
    MESSAGE(STATUS "Compiling on Cygwin")
    ADD_DEFINITIONS(-DCYGWIN)
  ELSE(CYGWIN)
    ADD_DEFINITIONS(-DWINDOWS)
    MESSAGE(STATUS "Compiling on Windows")
  ENDIF(CYGWIN)
  IF(MSVC)
    MESSAGE(STATUS "Compiling on Windows (MSVC)")
    ADD_DEFINITIONS(-DNOMINMAX)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
  ENDIF(MSVC)
ELSEIF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ELSEIF(APPLE)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on OSX")
ENDIF(WIN32)

# For building the CHOLMOD / CSPARSE solvers
FIND_PACKAGE(SuiteSparse)
IF(CSPARSE_FOUND)
  SET(BUILD_CSPARSE OFF CACHE BOOL "Build local CSparse library")
ELSE(CSPARSE_FOUND)
  SET(BUILD_CSPARSE ON CACHE BOOL "Build local CSparse library")
  SET(CSPARSE_FOUND TRUE)
ENDIF(CSPARSE_FOUND)

# OpenGL is used in the draw actions for the different types, as well
# as for creating the GUI itself
FIND_PACKAGE(OpenGL)
SET(G2O_FRONTEND_USE_OPENGL ON CACHE BOOL "Build g2o with OpenGL support for visualization")
IF (OPENGL_FOUND AND G2O_USE_OPENGL)
  SET (G2O_FRONTEND_HAVE_OPENGL 1)
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE})
  MESSAGE(STATUS "Compiling with OpenGL support")
ENDIF(OPENGL_FOUND AND G2O_USE_OPENGL)

# For building the GUI
FIND_PACKAGE(QGLViewer)

# shall we build the core apps using the library
SET(G2O_FRONTEND_BUILD_APPS ON CACHE BOOL "Build g2o apps")
IF(G2O_FRONTEND_BUILD_APPS)
  MESSAGE(STATUS "Compiling g2o apps")
ENDIF(G2O_FRONTEND_BUILD_APPS)

# Compiler specific options for gcc
IF(CMAKE_COMPILER_IS_GNUCXX)
  # Generic settings for optimisation
  SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse4") 
  SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -msse4") 

  # OS X
  #IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    #SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}") 
    #SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}") 
  #ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
  # Linux
  IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native") 
    SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native") 
  ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  # activate warnings !!!
  SET(g2o_frontend_C_FLAGS "${g2o_frontend_C_FLAGS} -Wall -W")
  SET(g2o_frontend_CXX_FLAGS "${g2o_frontend_CXX_FLAGS} -Wall -W")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# specifying compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_frontend_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_frontend_C_FLAGS}")

# Find Eigen3
FIND_PACKAGE(Eigen3 3.1.2 REQUIRED)

FIND_PACKAGE(Threads REQUIRED)

# Find g2o
FIND_PACKAGE(G2O REQUIRED)
if (G2O_FOUND)
  include_directories(${G2O_INCLUDE_DIR})
endif(G2O_FOUND)

FIND_PACKAGE(OpenMP)
if(OPENMP_FOUND)
 set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
 set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -D_PWN_USE_OPENMP_ -DEIGEN_DONT_PARALLELIZE")
 set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif(OPENMP_FOUND)

# Find CUDA
find_package(CUDA)
if(CUDA_FOUND)
  set(CUDA_NVCC_FLAGS "-gencode arch=compute_20,code=sm_20")
  #set(CUDA_NVCC_FLAGS "-gencode arch=compute_30,code=sm_30")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_PWN_USE_CUDA_")
endif(CUDA_FOUND)

# Find PCL
set(PCL_DIR $ENV{PCL_ROOT})
find_package(PCL 1.6 REQUIRED) #test malcom
IF(PCL_FOUND)
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
ENDIF(PCL_FOUND)

FIND_PACKAGE( OpenCV 2.4.2 REQUIRED )

# Set up the top-level include directories
INCLUDE_DIRECTORIES(${g2o_frontend_SOURCE_DIR} ${G2O_FRONTEND_G2O_INCLUDE} ${G2O_FRONTEND_EIGEN3_INCLUDE})

# Generate config.h
SET(G2O_FRONTEND_HAVE_CHOLMOD ${CHOLMOD_FOUND})
SET(G2O_FRONTEND_HAVE_CSPARSE ${CSPARSE_FOUND})
SET(G2O_FRONTEND_SHARED_LIBS ${BUILD_SHARED_LIBS})
configure_file(config.h.in ${PROJECT_BINARY_DIR}/g2o_frontend/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o_frontend/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o_frontend)
PROJECT( g2o_frontend_proj )

IF(EXISTS "${g2o_frontend_SOURCE_DIR}/g2o_frontend/traversability")
  MESSAGE("TRAVERSABILITY FOUND")
  SET(TRAVERSABILITY_FOUND true)
ELSE()
  MESSAGE("TRAVERSABILITY NOT FOUND")
  SET(TRAVERSABILITY_FOUND false)
ENDIF()

# Include the subdirectories
ADD_SUBDIRECTORY(g2o_frontend)

IF(EXISTS "${g2o_frontend_SOURCE_DIR}/g2o_frontend/traversability")
  SET(TRAVERSABILITY_FOUND true)
ELSE()
  SET(TRAVERSABILITY_FOUND false)
ENDIF()
