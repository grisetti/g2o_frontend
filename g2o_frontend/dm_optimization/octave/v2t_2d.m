# converts a transformation vector into an isometry
# x: the transformation vector [tx ty teta]
# X: the 3x3 transformation matrix

function X = v2t_2d(x)
  X = eye(3);
  X(1:2,3) = x(1:2,1);  
  X(1:2,1:2) = [cos(x(3,1)) -sin(x(3,1)); sin(x(3,1)) cos(x(3,1))];
end