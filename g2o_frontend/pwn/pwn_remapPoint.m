# remaps a point with normal w.r.t a transformation x
# p: the comumn vector of the point with a normal [tx ty tz nx ny nz]' the normal should have unit norm
# X: the 4x4 isometry of the transformation
# p2: the transformed points

function p2 = pwn_remapPoint(X,p)
  p2 = zeros(6,1);
  p2(1:3,1) = X(1:3,1:3)*p(1:3,1)+X(1:3,4);
  p2(4:6,1) = X(1:3,1:3)*p(4:6,1);
endfunction