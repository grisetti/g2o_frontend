#computes the solution for one iteration of least squares to align a set of lines onto another
#Li: the 6xN matrix of reference lines in plueker coords
#Lj: the 6xN matrix of current lines in plueker coords
#Omega: the information matrix of the lines, assumend to be the same for all
#X: the isometry of the transformation
#
# Xnew: the new transformation
# err: the chi2 at the previous iteration

function [Xnew, err]= line_iteration(Li, Lj, Omega, X)
  b = zeros(6,1);
  H = zeros(6,6);
  R = X(1:3, 1:3);
  t = X(1:3, 4);
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
    ljRemapped = line_remapPluecker(X,lj);
    # disp("ljremapped");
    # disp(ljRemapped');
    eij = li_ - ljRemapped;
    # disp("eij");
    # disp(eij');
    Jij_n = - line_jacobian_numeric(X, lj);
    Jij_a = - line_jacobian(X, lj);
    if (norm(Jij_n-Jij_a)>1e-9)
      disp("************** difference ************** ");
      disp(" norm:");
      disp(norm(Jij_n-Jij_a));
      disp(" Jn:");
      disp (Jij_n);
      disp(" Ja:");
      disp (Jij_a);
      disp(" Jn-Ja:");
      disp(Jij_n-Jij_a);
    endif;
    Jij=Jij_n;
    bij = Jij' * Omega * eij;
    Hij = Jij' * Omega * Jij;
    b += bij;
    H += Hij;
    err += eij' * Omega * eij;
  endfor
  dx = -H\b;
  dX = v2t(dx);
  Xnew  = X*dX;
endfunction