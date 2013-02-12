# remaps a cartesian line w.r.t a transformation x
# cl: the cartesian rep of the line [nx ny rho]
# X: the 3x3 isometry of the transformation
# tcl: the cartesian rep of the line [nx ny rho]

function tcl = line2d_remapCartesian(X, cl)
  tcl=cl;
  tcl(1:2,1) = X(1:2,1:2)*cl(1:2,1);
  tcl(3,1) +=  X(1:2,3)'*tcl(1:2,1);
endfunction