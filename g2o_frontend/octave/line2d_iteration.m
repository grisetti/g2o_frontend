# computes the solution for one iteration of least squares to align a set of lines2d onto another
# Li: the 3xN matrix of reference lines in cartesian coords
# Lj: the 3xN matrix of current lines in cartesian coords
# Omega: the information matrix of the lines, assumend to be the same for all
# X: the isometry of the transformation
#
# Xnew: the new transformation
# err: the chi2 at the previous iteration

function [Xnew, err]= line2d_iteration(Li, Lj, Omega, X)
  b = zeros(3,1);
  H = zeros(3,3);
  R = X(1:2, 1:2);
  t = X(1:2, 3);
  err = 0;
  for i = [1:size(Li)(2)]
    # disp("line: ");
    # disp(i);
    li_ = Li(:,i);
    lj =  Lj(:,i);
    # disp("li");
    # disp (li_');
    # disp("lj");
    # disp(lj');
    ljRemapped = line2d_remapCartesian(X,lj);
    # disp("ljremapped");
    # disp(ljRemapped');
    eij = li_ - ljRemapped;
    # disp("eij");
    # disp(eij');
    # Jij_n = - line2d_jacobian_numeric(X, lj);
    Jij = - line2d_jacobian_numeric(X, lj);
    bij = Jij' * Omega * eij;
    Hij = Jij' * Omega * Jij;
    b += bij;
    H += Hij;
    err += eij' * Omega * eij;
  endfor
  dx = -H\b;
  dX = v2t_2d(dx);
  Xnew  = X*dX;
endfunction