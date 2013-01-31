# computes the joacobian of the line2d remapping w.r.t an incremental transformation on a manifold
# X: the current remapping point
# pl: the pluecker line

function J = line2d_jacobian_numeric(X, pl)
  # compute the remapping matrix
  Ju = zeros(3,3);
  Jl = zeros(3,3);
  delta=1e-9;
  iDelta = .5/delta;
  for (i = [1:3])
    dxu=zeros(3,1);
    dxl=zeros(3,1);
    dxl(i)=-delta;
    dxu(i)=+delta;
    Ju(:,i)=line2d_remapCartesian(X*v2t_2d(dxu),pl);
    Jl(:,i)=line2d_remapCartesian(X*v2t_2d(dxl),pl);
  endfor
  J = (Ju-Jl)*iDelta;
endfunction;