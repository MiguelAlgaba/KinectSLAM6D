# remove runtime generated files
rm -rf results;
mkdir results;
cd results;
mkdir graphs;
mkdir matrices;
mkdir pcd_files;
cd ..;
cp matlab/plotPose.m results/;
cp matlab/plot_odometry.m results/;
cp matlab/plot_odometry_path.m results/;

