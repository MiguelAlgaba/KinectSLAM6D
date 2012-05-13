#Remove ANN binaries and created libraries
cd gicp/ann_1.1.1
cd ann2fig
rm ann2fig.o
cd .. #Back to ann_1.1.1 root directory
rm -rf bin
mkdir bin
rm -rf lib
mkdir lib
cd sample
rm ann_sample.o
cd .. #Back to ann_1.1.1 root directory
cd src
rm ANN.o
rm bd_fix_rad_search.o
rm bd_pr_search.o
rm bd_search.o
rm bd_tree.o
rm brute.o
rm kd_dump.o
rm kd_fix_rad_search.o
rm kd_pr_search.o
rm kd_search.o
rm kd_split.o
rm kd_tree.o
rm kd_util.o
rm perf.o
cd .. #Back to ann_1.1.1 root directory
cd test
rm ann_test.o
rm rand.o
cd .. #Back to ann_1.1.1 root directory
cd .. #Back to gicp root directory
cd .. #Back to KinectSLAM6D root directory

#Remove gicp binaries and created libraries
cd gicp
rm bfgs_funcs.o
rm gicp.o
rm libgicp.a
rm optimize.o
rm scan.o
rm scan2ascii
rm scan2ascii.o
rm test_gicp
rm test_gicp.o
rm transform.o
cd .. #Back to KinectSLAM6D root directory

#Remove g2o binaries and created libraries
cd g2o/trunk
rm -rf bin
mkdir bin
rm -rf build
mkdir build
rm -rf lib
mkdir lib
cd .. 
cd .. #Back to KinectSLAM6D root directory

