#computes the joacobian of the line remapping w.r.t an incremental transformation on a manifold
#X: the current remapping point
#pl: the pluecker line

function J = plane_jacobian_numeric(X, pl)
  #compute the remapping matrix
  Ju = zeros(4,6);
  Jl = zeros(4,6);
  delta=1e-9;
  iDelta = .5/delta;
  for (i = [1:6])
    dxu=zeros(6,1);
    dxl=zeros(6,1);
    dxl(i)=-delta;
    dxu(i)=+delta;
    Ju(:,i)=plane_remapCartesian(X*v2t(dxu),pl);
    Jl(:,i)=plane_remapCartesian(X*v2t(dxl),pl);
  endfor
  J = (Ju-Jl)*iDelta;
endfunction;