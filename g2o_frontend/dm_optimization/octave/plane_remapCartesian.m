# remaps a cartesian line w.r.t a transformation x
# cl: the cartesian rep of the line [p0x p0y p0z dir_x dir_y dir_z]
# X: the 4x4 isometry of the transformation
# tcl: the cartesian rep of the line [p0x p0y p0z dir_x dir_y dir_z]

function tpl = plane_remapCartesian(X, pl)
  # normalize the direction to the unit vector
  tpl=pl;
  tpl(1:3,1) = X(1:3,1:3)*pl(1:3,1);
  tpl(4,1) -=  X(1:3,4)'*tpl(1:3,1);
endfunction