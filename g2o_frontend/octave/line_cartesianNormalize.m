#normalizes the cartesian representation of a line
# cl: the cartesian rep of the line [ p0x p0y p0z dir_x dir_y dir_z]
# ncl: the cartesian line with a  unit vector as direction vector, and p0 set to the point in the line closest to the origin.

function ncl = line_cartesianNormalize(cl)
  # normalize the direction to the unit vector
  ncl=zeros(6,1);
  ncl(4:6) = cl(4:6)/norm(cl(4:6));
  # set p0 to the point closest to the origin
  ncl(1:3) = cl(1:3) - ncl(4:6) * dot(ncl(4:6),cl(1:3));
endfunction