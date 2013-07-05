# remaps a cartesian point w.r.t a transformation x
# cp: the cartesian rep of the point [px py]
# X: the 3x3 isometry of the transformation
# tcp: the cartesian rep of the point [px py]

function tcp = point2d_remapCartesian(X, cp)
  tcp=cp;
  tcp(1:2,1) = X(1:2,1:2)*cp(1:2,1) + X(1:2,3);
  tcp=tcp';
endfunction
