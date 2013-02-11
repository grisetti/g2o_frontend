#computes the full jacobianof the line remapping w.r.t an incremental
#transformation on a manifold AND a perturbation of the lines in minimal
#coords 
#X: the current remapping point
#pl: the pluecker line

function J = line_jacobian_numeric_full(X, pl)
  #compute the remapping matrix
  Ju = zeros(7,12);
  Jl = zeros(7,12);
  delta=1e-9;
  iDelta = .5/delta;
  for (i = [1:6])
    dxu=zeros(6,1);
    dxl=zeros(6,1);
    dxl(i)=-delta;
    dxu(i)=+delta;
    Ju(1:6,i)=line_remapPluecker(X*v2t(dxu),pl);
    Jl(1:6,i)=line_remapPluecker(X*v2t(dxl),pl);
  endfor
  for (i = [7:12])
    dxu=zeros(6,1);
    dxl=zeros(6,1);
    dxl(i-6)=-delta;
    dxu(i-6)=+delta;
    Ju(1:6,i)=line_remapPluecker(X,line_plueckerNormalize(pl+dxu));
    Ju(7,i) = (pl+dxu)(4:6)'*(pl+dxu)(4:6)-1;
    Jl(1:6,i)=line_remapPluecker(X,line_plueckerNormalize(pl+dxl));
    Jl(7,i) = (pl+dxl)(4:6)'*(pl+dxl)(4:6)-1;
  endfor
  J = (Ju-Jl)*iDelta;
endfunction;