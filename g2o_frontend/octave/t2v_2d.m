# converts an isometry to transformation vector 
# X: the 3x3 transformation matrix
# x: the transformation vector [tx ty theta]

function x = t2v_2d(X)
  x=zeros(3,1);
  x(1:2,:) = X(1:2,3);
  x(3,1) = atan2(X(2,1),X(1,1));
end