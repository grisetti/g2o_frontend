# remaps a cartesian line w.r.t a transformation x
# cl: the cartesian rep of the line [p0x p0y p0z dir_x dir_y dir_z]
# X: the 4x4 isometry of the transformation
# tcl: the cartesian rep of the line [p0x p0y p0z dir_x dir_y dir_z]

function tcl = line_remapCartesian(X, cl)
  # normalize the direction to the unit vector
  tcl=cl;
  tcl(1:3,1) = X(1:3,1:3)*tcl(1:3,1)+X(1:3,4);
  tcl(4:6,1) = X(1:3,1:3)*tcl(4:6,1);
  tcl = line_cartesianNormalize(tcl);
endfunction