function rpy = mat2rpy(mat) 
%MAT2RPY converts Rotation matrix to euler zyx angles rpy : roll, pitch, yaw
rpy = zeros(3, 1);
rpy(2) = -asin(mat(3, 1));
% Check if cos(pitch) is close to zero :
if abs(cos(rpy(2))) < 0.00001 
    disp('Pitch is very close to 90 degrees');
    rpy(1) = atan2(mat(1, 2), mat(1, 3));
    rpy(3) = 0;
    return;
end
rpy(1) = atan2(mat(3, 2), mat(3, 3));
rpy(3) = atan2(mat(2, 1), mat(1, 1));
end
