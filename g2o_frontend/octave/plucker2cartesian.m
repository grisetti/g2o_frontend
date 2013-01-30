# converts an isometry to transformation vector 
# X: the 4x4 transformation matrix
# x: the transformation vector [tx ty tz qx qy qz]

function lp = pluckerToCartesian(lc)
  x=zeros(6,1);
  x(1:3,:) = X(1:3,4);
  x(4:6,1) = mat2quat(X(1:3,1:3));
end