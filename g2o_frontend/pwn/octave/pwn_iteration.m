#computes the solution for one iteration of least squares to align a set of points onto another
#Pi: the 6xN matrix of reference points with normal
#Pj: the 6xN matrix of current points with normal
#Omega: the information matrix of the points, assumend to be the same for all
#X: the isometry of the transformation
#
# Xnew: the new transformation
# err: the chi2 at the previous iteration

function [Xnew, err]= pwn_iteration(Pi, Pj, Omega, X)
  b = zeros(6,1);
  H = zeros(6,6);
  R = X(1:3, 1:3);
  t = X(1:3, 4);
  err = 0;
  for i = [1:size(Pi)(2)]
    pi_ = Pi(:,i);
    pj = Pj(:,i);
    pjRemapped = pwn_remapPoint(X,pj);
    eij = pi_ - pjRemapped;
    Jij = - pwn_jacobian(X, pj);
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