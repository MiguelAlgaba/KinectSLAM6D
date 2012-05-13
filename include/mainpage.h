/*!\mainpage Documentation Overview
 *
 * \section intro_sec Introduction
 *
 * This is the documentation of the Kinect-RGBD-GraphSLAM6D Final Year Project. This project integrates state-of-the-art algorithms to generate 3D maps using a hand-held Kinect sensor. The provided solution is intended to take advantage of the two main sensors that the Microsoft Kinect device contains, which are the RGB camera and the range sensor. A first approximation to the trajectory of the sensor is estimated using a pairwise alignment approach using consecutive point clouds. To avoid cummulative error this project uses a GraphSLAM approach, that mitigates the drift in the trajectory and therefore, inconsistencies in the map.
 * \image html  KinectSLAMPipelineEnglish.svg
 *
 * \section dependencies_sec Dependencies
 * This project integrates several open-source libraries to build the whole solution. The main dependencies are:
 *    - OpenCV: http://opencv.willowgarage.com/wiki/ 
 *    - PCL: http://pointclouds.org/
 *    - MRPT: http://www.mrpt.org/
 *    - GICP: http://www.stanford.edu/~avsegal/generalized_icp.html
 *    - G2O: http://openslam.org/g2o
 *    - Eigen: http://eigen.tuxfamily.org
 *    - Auxiliary libraries
 *         -# FLANN
 *         -# ANN
 *         -# GSL
 *         -# SuiteSparse
 *         -# OpenNI
 *         -# libfreenect
 *         -# Boost
 *         -# VTK 
 *         -# CUDA
 *
 * \section install_sec Installation
 * This project has been implemented and tested in Ubuntu 11.04. To compile the source code you need to install the dependencies first. After that, follow the following steps to compile the project.
 *    - Compile the GICP library.
 *         -# Download the GICP library inside the KinectSLAM6D root directory. The source code of the GICP library can be found in the following link. \n
 *            http://www.stanford.edu/~avsegal/generalized_icp.html
 *         -# Compile ANN library using the following commands: \n 
\verbatim 
cd KinectSLAM6D/gicp/ann_1.1.1
make linux-g++
\endverbatim
 *         -# Compile the GICP library using the following commands: \n
\verbatim 
cd KinectSLAM6D/gicp
make
\endverbatim
 *
 *    - Get the G2O library (tested with revision 23).
 *         -# Download the g2o library inside the KinectSLAM6D root directory using the following commands.
\verbatim 
cd KinectSLAM6D
svn co https://svn.openslam.org/data/svn/g2o
\endverbatim
 *
 *    - Generate the Code::Blocks project.
 *         -# Open CMake. 
 *         -# Set the source directory to KinectSLAM6D and the build directory to KinectSLAM6D/build.
 *         -# Set OpenCV_DIR and MRPT_DIR to the OpenCV and MRPT build directories respectively.
 *         -# Configure.
 *         -# Generate.
 *
 *    - Compile the KinectSLAM6D project.
 *         -# Open the KinectSLAM6D/build/KinectSLAM6D.cbp project.
 *         -# Compile.
 * 
 *    - Install CVPR tools (optional).
 *         -# Install the CVPR tools dependencies.	   
 *         -# Download the CVPR tools in the KinectSLAM6D/tools directory. The set of tools can be found in the following link. \n
 *            https://cvpr.in.tum.de/data/datasets/rgbd-dataset/tools 
\verbatim 
cd KinectSLAM6D/tools
svn co https://svncvpr.in.tum.de/cvpr-ros-pkg/trunk/rgbd_benchmark/rgbd_benchmark_tools/src/rgbd_benchmark_tools
\endverbatim       
 *
 * \section usage_sec Software usage
 * 
 * After compiling the project, two executables should appear in the KinectSLAM6D/build directory. The program PairwiseAlignmentSteps shows visually the steps performed in the stage of Pairwise Alignment. The program Kinect6DSLAM generates 3D maps from the provided RGB-D data. The RGB-D data can be provided using three different interfaces: two of them grabs RGB-D frames directly from the Kinect sensor (online). These two interfaces use the MRPT or PCL libraries to access the Kinect data. The last interface grabs RGB-D frames from .rawlog datasets using the MRPT library. In the following link there are some RGB-D datasets (.rawlog) that can be used to test the programs. To change between different configurations, set the proper #define's in the kinect6DSLAM.cpp and PairwiseAlignmentSteps.cpp files.
 *
 * http://www.mrpt.org/robotic_datasets
 * 
 * The original datasets can be found in the following link. However they are not prepared to be used in this sofware.
 *
 * http://cvpr.in.tum.de/data/datasets/rgbd-dataset
 *
 * These RGB-D datasets also contain a very precise ground-truth that can be very useful to evaluate different configurations. In the previous link, the CVPR group also provides a set of tools to measure the error between the estimated trajectory and the real trajectory of the ground-truth.	
 *
 * \subsection PairwiseAlignmentSteps
 * This program performs Pairwise Alignment between two RGB-D frames selected by the user. This program aims to show visually the different steps that occur during the Pairwise Alignment process. This program requires the user to select two RGB-D frames of the sequence data. Press "enter" in the OpenCV window to select an RGB-D frame. The program stores the results of the alignment in the KinectSLAM6D/results directory.
 *
\verbatim 
cd KinectSLAM6D/build
./PairwiseAlignmentSteps [* <rgbd_dataset>.rawlog]
\endverbatim
 * [*] Optional parameter to provide a RGB-D dataset.
 *
 * \image html  PairwiseAlignmentStepsOpenCV.png "OpenCV window that shows the 2D features over the image data. Press enter over this window to grab a frame."
 * \image html  PairwiseAlignmentStepsPCL.png "PCL window that shows the 3D features that correspond to the 2D features."
  * \image html  PairwiseAlignmentStepsResult.png "PCL window that shows the Pairwise Alignment result. Red: first point cloud. Blue: second point cloud. The result is stored in a .pcd file; to open the .pcd file use the pcd_viewer tool of the PCL library."
 *
 * \subsection Kinect6DSLAM
 * This program generates a 3D map from the provided secuence of RGB-D frames. This program requires the user to press enter to stop grabbing RGB-D frames. The program stores the results of the alignment in the KinectSLAM6D/results directory. The program generates .pcd files that contain a global point cloud representing the 3D map. These files are generated in the KinectSLAM6D/results/pcd_files directory. The estimated trajectory files and GraphSLAM graphs are generated in the KinectSLAM6D/results/graphs directory.
 *
\verbatim 
cd KinectSLAM6D/build
./KinectSLAM6D [* <rgbd_dataset>.rawlog] [** <grount_truth>.txt]
\endverbatim
 * [*] Optional parameter to provide a RGB-D dataset. \n
 * [**] Optional parameter to provide the ground-truth of the RGB-D dataset. (requires the CVPR tools and depencencies installed)
 *
 * \image html  KinectSLAM6DOpenCV.png "OpenCV window that shows the 2D correspondences between the current and previous frame. Press enter over this window to stop grabbing frames."
 *
 * \image html  KinectSLAM6DResult.png "PCL window that shows the generated global map. The result is stored in a .pcd file; to open the .pcd file use the pcd_viewer tool of the PCL library."
 *
 * \image html  KinectSLAM6DTrajectory.svg "Estimated trajectory compared to the ground-truth trajectory. This requires both the RGB-D dataset and ground-truth files, as well as the CVPR tools."
 *
 * \author    Miguel Algaba Borrego
 * \n http://thecomputervision.blogspot.com/
 */
