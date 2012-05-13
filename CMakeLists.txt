PROJECT(KinectSLAM6D)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
if(COMMAND cmake_policy)
      cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
endif(COMMAND cmake_policy)

# Set the output directory for the build executables
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/build)

#Add here your project dependencies
FIND_PACKAGE(MRPT REQUIRED hwdrivers maps graphslam) #Add here your project dependencies
FIND_PACKAGE(PCL REQUIRED) 
FIND_PACKAGE(OpenCV REQUIRED )

INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})

# Required by StanfordGICP
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake) 
FIND_PACKAGE(GSL REQUIRED)
FIND_PACKAGE(Boost COMPONENTS system program_options REQUIRED)
include_directories(${PROJECT_SOURCE_DIR}/gicp)
include_directories(${PROJECT_SOURCE_DIR}/gicp/ann_1.1.1/include/ANN)

# G2O library

# Set up the top-level include directories
SET( G2O_INCLUDE ${PROJECT_SOURCE_DIR}/EXTERNAL/g2o CACHE PATH "Directory of G2O")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${G2O_INCLUDE})

# Add g2o lib dir
LINK_DIRECTORIES( ${LINK_DIRECTORIES} "${G2O_INCLUDE}/lib" )

#Generate config.h 
configure_file(g2o/trunk/config.h.in ${PROJECT_BINARY_DIR}/g2o/config.h)
include_directories(${PROJECT_BINARY_DIR})
INSTALL(FILES ${PROJECT_BINARY_DIR}/g2o/config.h DESTINATION ${CMAKE_INSTALL_PREFIX}/include/g2o)

# Include the subdirectories
ADD_SUBDIRECTORY(g2o/trunk)

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR}) #You can use CPARSE or CHOLMOD
INCLUDE_DIRECTORIES(${CHOLMOD_INCLUDE_DIR}) 

set(G2O_DIR ${PROJECT_SOURCE_DIR}/g2o/trunk)
include_directories(${G2O_DIR})
link_directories(${G2O_DIR}/lib)


# Declare the target (an executable)
ADD_EXECUTABLE(Kinect6DSLAM kinect6DSLAM.cpp
./include/KinectGrabber.h
./include/KinectGrabber_OpenNI.h
./src/KinectGrabber_OpenNI.cpp
./include/KinectGrabber_Rawlog.h
./src/KinectGrabber_Rawlog.cpp
./include/KinectGrabber_Rawlog2.h
./src/KinectGrabber_Rawlog2.cpp
./include/KinectGrabber_MRPT.h
./src/KinectGrabber_MRPT.cpp
./include/VisualFeatureDescriptorExtractor.h
./include/VisualFeatureDescriptorExtractor_SURF_GPU.h
./include/VisualFeatureDescriptorExtractor_Generic.h
./include/VisualFeatureDescriptorExtractor_ORB.h
./include/VisualFeatureMatcher.h
./include/VisualFeatureMatcher_Generic.h
./include/PointCloudViewer.h
./include/PointCloudViewer_MRPT.h
./src/VisualFeatureDescriptorExtractor_SURF_GPU.cpp
./src/VisualFeatureDescriptorExtractor_Generic.cpp
./src/VisualFeatureDescriptorExtractor_ORB.cpp
./src/VisualFeatureMatcher_Generic.cpp
./src/PointCloudViewer_MRPT.cpp
./include/Visual3DRigidTransformationEstimator.h
./include/Visual3DRigidTransformationEstimator_SVD.h
./src/Visual3DRigidTransformationEstimator_SVD.cpp
./include/Visual3DRigidTransformationEstimator_RANSAC.h
./src/Visual3DRigidTransformationEstimator_RANSAC.cpp
./include/ICPPoseRefiner.h
./include/ICPPoseRefiner_PCL.h
./src/ICPPoseRefiner_PCL.cpp
./include/ICPPoseRefiner_StanfordGICP.h
./src/ICPPoseRefiner_StanfordGICP.cpp
./include/Miscellaneous.h
./src/Miscellaneous.cpp
./include/FrameRGBD.h
./src/FrameRGBD.cpp
./include/PointCloudDownsampler.h
./src/PointCloudDownsampler.cpp
./include/KeyframeLoopDetector.h
./src/KeyframeLoopDetector.cpp
./include/GraphOptimizer.h
./include/GraphOptimizer_MRPT.h
./src/GraphOptimizer_MRPT.cpp
./include/GraphOptimizer_G2O.h
./src/GraphOptimizer_G2O.cpp
./include/CKinect2DRawlog.h
./src/CKinect2DRawlog.cpp
)

TARGET_LINK_LIBRARIES(Kinect6DSLAM ${MRPT_LIBS}
				   ${PCL_LIBRARIES} 
				   ${OpenCV_LIBS}
		      		   ${Boost_LIBRARIES}
				   #GICP
                                   ${GSL_LIBRARIES} 
                                   ${PROJECT_SOURCE_DIR}/gicp/ann_1.1.1/lib/libANN.a
                                   ${PROJECT_SOURCE_DIR}/gicp/libgicp.a
				   #G2O
				   core  math_groups types_slam3d
		      	    	   solver_csparse #You can use CPARSE or CHOLMOD
		      	           solver_cholmod ${CHOLMOD_LIBRARIES}
				   )

# Declare the target (an executable)
ADD_EXECUTABLE(PairwiseAlignmentSteps PairwiseAlignmentSteps.cpp
./include/KinectGrabber.h
./include/KinectGrabber_OpenNI.h
./src/KinectGrabber_OpenNI.cpp
./include/KinectGrabber_Rawlog.h
./src/KinectGrabber_Rawlog.cpp
./include/KinectGrabber_MRPT.h
./src/KinectGrabber_MRPT.cpp
./include/VisualFeatureDescriptorExtractor.h
./include/VisualFeatureDescriptorExtractor_SURF_GPU.h
./include/VisualFeatureDescriptorExtractor_Generic.h
./include/VisualFeatureDescriptorExtractor_ORB.h
./include/VisualFeatureMatcher.h
./include/VisualFeatureMatcher_Generic.h
./include/PointCloudViewer.h
./include/PointCloudViewer_MRPT.h
./src/VisualFeatureDescriptorExtractor_SURF_GPU.cpp
./src/VisualFeatureDescriptorExtractor_Generic.cpp
./src/VisualFeatureDescriptorExtractor_ORB.cpp
./src/VisualFeatureMatcher_Generic.cpp
./src/PointCloudViewer_MRPT.cpp
./include/Visual3DRigidTransformationEstimator.h
./include/Visual3DRigidTransformationEstimator_SVD.h
./src/Visual3DRigidTransformationEstimator_SVD.cpp
./include/Visual3DRigidTransformationEstimator_RANSAC.h
./src/Visual3DRigidTransformationEstimator_RANSAC.cpp
./include/ICPPoseRefiner.h
./include/ICPPoseRefiner_PCL.h
./src/ICPPoseRefiner_PCL.cpp
./include/ICPPoseRefiner_StanfordGICP.h
./src/ICPPoseRefiner_StanfordGICP.cpp
./include/Miscellaneous.h
./src/Miscellaneous.cpp
./include/FrameRGBD.h
./src/FrameRGBD.cpp
./include/PointCloudDownsampler.h
./src/PointCloudDownsampler.cpp
)

TARGET_LINK_LIBRARIES(PairwiseAlignmentSteps ${MRPT_LIBS}
				   ${PCL_LIBRARIES} 
				   ${OpenCV_LIBS}
		      		   ${Boost_LIBRARIES}
				   #GICP
                                   ${GSL_LIBRARIES} 
                                   ${PROJECT_SOURCE_DIR}/gicp/ann_1.1.1/lib/libANN.a
                                   ${PROJECT_SOURCE_DIR}/gicp/libgicp.a
				   )

# Set optimized building:
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")


