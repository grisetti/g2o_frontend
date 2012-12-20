# converts a line expressed as a direction vector and an initial point into pluecker coordinates
# cartesian: the cartesian rep of the line [p0x p0y p0z dir_x dir_y dir_z]
# pluecker: the 4x4 transformation matrix
function pluecker = line_cartesian2pluecker(cartesian)
  cartesian = line_cartesianNormalize(cartesian);
  pluecker=zeros(6,1);
  pluecker(1:3) = cross(cartesian(1:3),cartesian(4:6)+cartesian(1:3));
  pluecker(4:6) = cartesian(4:6);
end