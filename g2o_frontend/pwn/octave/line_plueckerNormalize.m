#normalizes the pluecker representation of a line, by setting the direction part to 
#the unit be one

function npl = line_plueckerNormalize(pl)
  # normalize the direction to the unit vector
  npl = pl* (1./norm(pl(4:6)));
endfunction