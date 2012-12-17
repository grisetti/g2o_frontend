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
    eij = eij(1:5);
    # disp("eij");
    # disp(eij');
    #Jij_n = - line_jacobian_numeric(X, lj);
    Jij_a = - line_jacobian(X, lj);

    fJij_a = line_jacobian_full(X, lj);
    fJij_n = line_jacobian_numeric_full(X, lj);

    if (norm(fJij_n-fJij_a)>1)
      disp("************** difference ************** ");
      disp(" norm:");
      disp(norm(fJij_n-fJij_a));
      disp(" lj:");
      disp(lj');
      disp(" changeDir:");
      disp(lj(4:5)'*ljRemapped(4:5));
      disp(" ljRemapped:");
      disp(ljRemapped');
      disp(" fJn:");
      disp (fJij_n);
      disp(" fJa:");
      disp (fJij_a);
      disp(" fJn-fJa:");
      disp(fJij_n-fJij_a);
    endif;

    Jij=Jij_a;
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