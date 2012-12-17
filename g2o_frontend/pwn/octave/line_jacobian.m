#computes the joacobian of the line remapping w.r.t an incremental transformation on a manifold
#X: the current remapping point
#pl: the pluecker line

function J = line_jacobian(X, pl)
  #compute the remapping matrix
  A = zeros(6,6);
  A(1:3, 1:3) = X(1:3,1:3);
  A(1:3, 4:6) = skew(X(1:3,4))*X(1:3,1:3);
  A(4:6, 4:6) = X(1:3,1:3);
  plRemapped = A*pl;
  # compute the differential part (some math behind this)
  D = zeros(6,6);
  D(1:3,1:3) = -skew(pl(4:6)) ;
  D(1:3,4:6) = -2*skew(pl(1:3));
  D(4:6,4:6) = -2*skew(pl(4:6));
  J = A*D* sign(plRemapped(6));
  J = J(1:5,1:6);
endfunction;