% normalizes the cartesian representation of a line
% cl: the cartesian rep of the line [ p0x p0y dir_x dir_y]
% ncl: the cartesian line with a  unit vector as direction vector, and p0 set to the point in the line closest to the origin.

function ncl = line2d_cartesianNormalize(cl)
  # normalize the direction to the unit vector
  ncl=zeros(4,1);
  ncl(3:4) = cl(3:4)/norm(cl(3:4));
  # set p0 to the point closest to the origin
  ncl(1:2) = cl(1:2) - ncl(3:4) * dot(ncl(3:4),cl(1:2));
endfunction