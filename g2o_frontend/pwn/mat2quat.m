# converts a rotation matrix a unit normalized quaternion into 
# R: the rotation matrix
# q: the output quaternion [qx qy qz]

function q = mat2quat(R)
  q  = [(R(3,2)-R(2,3))/(2*sqrt(1+R(1,1)+R(2,2)+R(3,3)));
        (R(1,3)-R(3,1))/(2*sqrt(1+R(1,1)+R(2,2)+R(3,3)));
        (R(2,1)-R(1,2))/(2*sqrt(1+R(1,1)+R(2,2)+R(3,3)))];
end;
