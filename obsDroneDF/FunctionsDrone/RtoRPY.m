function [roll, pitch, yaw] = RtoRPY(R)
    yaw = atan2(-R(1, 2), R(2, 2));
    roll = atan2(R(3,2), sqrt( 1-R(3,2)^2 ) );
    pitch = atan2(-R(3,1), R(3,3));
end