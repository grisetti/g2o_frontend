#computes the solution for one iteration of least squares to align a set of lines onto another
#Li: the 6xN matrix of reference lines in plueker coords
#Lj: the 6xN matrix of current lines in plueker coords
#Omega: the information matrix of the lines, assumend to be the same for all
#X: the isometry of the transformation
#
# Xnew: the new transformation
# err: the chi2 at the previous iteration

function [Xnew, err]= line_iteration_full(L, Li, Lj, Omega, X)
  dim = size(L)(2)+6;
  b = zeros(dim,1);
  H = zeros(dim,dim);
  R = X(1:3, 1:3);
  t = X(1:3, 4);
  L=Li;
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
    eij=zeros(1,7);
    eij(1:6) = li_ - ljRemapped;
    eij(7) = lj(4:6)'*lj(4:6);
    # disp("eij");
    # disp(eij');
    #Jij_n = - line_jacobian_numeric(X, lj);
    Jij = - line_jacobian(X, lj);
    Jij=Jij_a;
    bij = Jij' * Omega * eij;
    Hij = Jij' * Omega * Jij;
    
    H(1:6,1:6) += Hij(1:6,1:6);
    H(1:6,i*6+1:(i+1)*6) += Hij(1:6,7:12);
    H(i*6+1:(i+1)*6,1:6) += Hij(7:12,1:6);
    H(i*6+1:(i+1)*6,i*6+1:(i+1)*6) += Hij(7:12,7:12);

    b(1:6) += bij(1:6);
    b(i*6+1:(i+1)*6) += bij;
  endfor
  dx = -H\b;
  dX = v2t(dx(1:6));
  
  Xnew  = X*dX;
  for i = [1:size(Li)(2)]
    L(:,i) = line_plueckerNormalize(pl)
  endfor
endfunction