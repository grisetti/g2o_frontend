# applies the transform x to a set of poiunts with normals
# P:  the input matrix of points
# X:  the 4x4 isometry of the transformation
# Pn: the transformed points
function Pn = pwn_remapPoints(P, X)
  Pn = P;
  for i = [1:size(P)(2)]
    Pn(:, i) =  pwn_remapPoint(X, Pn(:, i));
  endfor;
endfunction