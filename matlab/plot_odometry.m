clear;
clc;
clf;

load poses.mat;

for measureIndx=1:length(poses)

x=poses(measureIndx,1);
y=poses(measureIndx,2);
phi=poses(measureIndx,3);
plotPose(x,y,phi,0.05);

end
