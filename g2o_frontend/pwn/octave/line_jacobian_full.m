#computes the full jacobianof the line remapping w.r.t an incremental
#transformation on a manifold AND a perturbation of the lines in minimal
#coords 
#X: the current remapping point
#pl: the pluecker line

function J = line_jacobian_full(X, pl)
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
  Jp = A*D;#//* sign(plRemapped(6));
  
  ax = pl(1);
  ay = pl(2);
  az = pl(3);
  bx = pl(4);
  by = pl(5);
  bz = pl(6);
  ln = sqrt(bz^2+by^2+bx^2);
  Jll=[1/ln,0,0,-(ax*bx)/ln^3,-(ax*by)/ln^3,-(ax*bz)/ln^3;
       0,1/ln,0,-(ay*bx)/ln^3,-(ay*by)/ln^3,-(ay*bz)/ln^3;
       0,0,1/ln,-(az*bx)/ln^3,-(az*by)/ln^3,-(az*bz)/ln^3;
       0,0,0,-(bx^2-ln^2)/ln^3,-(bx*by)/ln^3,-(bx*bz)/ln^3;
       0,0,0,-(bx*by)/ln^3,-(by^2-ln^2)/ln^3,-(by*bz)/ln^3;
       0,0,0,-(bx*bz)/ln^3,-(by*bz)/ln^3,-(bz^2-ln^2)/ln^3];
  J=zeros(7,12);
  J(1:6,1:6)=Jp;
  J(1:6,7:12) = A*Jll;
  J(7,7:12) = [0,0,0,2*bx,2*by,2*bz];
endfunction
