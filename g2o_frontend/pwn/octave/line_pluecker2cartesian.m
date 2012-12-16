# converts a line expressed as a direction vector and an initial point into pluecker coordinates
# cartesian: the cartesian rep of the line [dir_x dir_y dir_z p0x p0y p0z]
# pluecker: the pluecker (normalized) representation
function cartesian = line_pluecker2cartesian(pluecker)
  cartesian=zeros(6,1);
  cartesian(4:6) = pluecker(4:6)*(1./norm(pluecker(4:6)));
  A=-skew(pluecker(4:6));
  cartesian(1:3) = pinv(A)*pluecker(1:3); #this does the pseudo inversion to find the minimum norm v
  #cartesian = line_cartesianNormalize(cartesian);
end