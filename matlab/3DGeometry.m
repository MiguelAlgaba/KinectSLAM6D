pose1 = pose(3,6,0,0,0,0) %First pose

pose2 = pose(4,10,-5,pi/2,0,0) %Second pose

p_local = point(1,2,1) %Local coordinates

p_1_global = pose1 * p_local %Point (1,2,1) of frame1 to global coordinates

p_2_global = pose2 * p_local %Point (1,2,1) of frame2 to global coordinates

T = inv(pose1) * pose2 %Relative rigid transformation

p_1_2 = T * p_local %Point (1,2,1) of frame2 seen from frame1

pose2_ = pose1 * T %Pose composition


