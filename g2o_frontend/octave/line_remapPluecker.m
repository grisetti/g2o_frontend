# remaps a cartesian line w.r.t a transformation x
# pl: the pluecker rep of the line [wx wy wz dir_x dir_y dir_z]
# X: the 4x4 isometry of the transformation
# tpl: the pluecker rep of the line [wx wy wz dir_x dir_y dir_z]

function tpl = line_remapPluecker(X, pl)
  # construct the pluecker transformation matrix
  A = zeros(6,6);
  A(1:3, 1:3) = X(1:3,1:3);
  A(1:3, 4:6) = skew(X(1:3,4))*X(1:3,1:3);
  A(4:6, 4:6) = X(1:3,1:3);
  tpl = line_plueckerNormalize(A*pl);
endfunction