#computes the full jacobianof the line remapping w.r.t an incremental
#transformation on a manifold AND a perturbation of the lines in minimal
#coords 
#X: the current remapping point
#pl: the pluecker line

function J = line_jacobian_numeric_full(X, pl)
  #compute the remapping matrix
  Ju = zeros(6,11);
  Jl = zeros(6,11);
  delta=1e-9;
  iDelta = .5/delta;
  for (i = [1:6])
    dxu=zeros(6,1);
    dxl=zeros(6,1);
    dxl(i)=-delta;
    dxu(i)=+delta;
    Ju(:,i)=line_remapPluecker(X*v2t(dxu),pl);
    Jl(:,i)=line_remapPluecker(X*v2t(dxl),pl);
  endfor
  for (i = [7:11])
    dxu=zeros(6,1);
    dxl=zeros(6,1);
    dxl(i-6)=-delta;
    dxl(6)=sqrt(1-dxl(4:5)'*dxl(4:5));
    dxu(i-6)=+delta;
    dxu(6)=sqrt(1-dxu(4:5)'*dxu(4:5));
    Ju(:,i)=line_remapPluecker(X,line_plueckerNormalize(pl+dxu));
    Jl(:,i)=line_remapPluecker(X,line_plueckerNormalize(pl+dxl));
  endfor
  J = (Ju-Jl)*iDelta;
  J=J(1:5,1:11);
endfunction;