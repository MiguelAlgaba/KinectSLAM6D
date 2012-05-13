function P = pose(x,y,z,yaw,pitch,roll)

    P = zeros(4,4);

    P(1,1) = cos(yaw) * cos(pitch);
    P(1,2) = cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll);
    P(1,3) = cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll);
    P(1,4) = x;

    P(2,1) = sin(yaw) * cos(pitch);
    P(2,2) = sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll);
    P(2,3) = sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll);
    P(2,4) = y;

    P(3,1) = -sin(pitch);
    P(3,2) = cos(pitch) * sin(roll);
    P(3,3) = cos(pitch) * cos(roll);
    P(3,4) = z;

    P(4,1) = 0;
    P(4,2) = 0;
    P(4,3) = 0;
    P(4,4) = 1;

end
