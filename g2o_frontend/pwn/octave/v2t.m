# converts a transformation vector into an isometry
# x: the transformation vector [tx ty tz qx qy qz]
# X: the 4x4 transformation matrix
function X = v2t(x)
  X = eye(4);
  X(1:3,4) = x(1:3,1);
  X(1:3,1:3) = quat2mat(x(4:6,1));
end