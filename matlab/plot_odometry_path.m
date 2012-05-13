clear;
clc;
clf;

load GT_path_vehicle.txt;

for measureIndx=1:length(GT_path_vehicle)

x=GT_path_vehicle(measureIndx,2);
y=GT_path_vehicle(measureIndx,3);
phi=GT_path_vehicle(measureIndx,5);
plotPose(x,y,phi,0.05);

end
