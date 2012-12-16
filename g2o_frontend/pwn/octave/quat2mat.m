# converts a unit normalized quaternion into a rotation matrix
# R: the rotation matrix
# q: the input quaternion [qx qy qz]

function R = quat2mat(q) 
  qx = q(1);
  qy = q(2);
  qz = q(3);
  qw = 0;
  if (q'*q <= 1)
    qw = sqrt(1 - q'*q);
  else
    qx = 0;
    qy = 0;
    qw = 0;
  endif;
  
  R = [
       qw*qw + qx*qx - qy*qy - qz*qz, 	 2*(qx*qy - qw*qz) ,  2*(qx*qz + qw*qy);
	 2*(qx*qy + qz*qw) , qw*qw - qx*qx + qy*qy - qz*qz,  2*(qy*qz - qx*qw);
	 2*(qx*qz - qy*qw) , 2*(qy*qz + qx*qw), qw*qw - qx*qx - qy*qy + qz*qz ];
  end;
