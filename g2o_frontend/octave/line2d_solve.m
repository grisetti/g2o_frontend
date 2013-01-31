# performs a sequence of iterations to find the transform that maps a set of lines in pluecker coordinates onto another
# Li: the 3xN matrix of reference lines in cartesian coords
# Lj: the 3xN matrix of current lines in cartesian coords
# Omega: the information matrix of the lines, assumend to be the same for all
# X: the isometry of the transformation
# n:     num of iterations
#
# returns:
# Xs:    an array containing the resulting transform at each iteration
# es:    an array containing the evolution of the error

function [Xs, es] = line2d_solve(Li, Lj, Omega, X, n)
  Xs = zeros(3, 3, n);
  es = zeros(1, n);
  for i = [1:n]
    disp("*******************************************");
    disp("iteration:");
    disp(i);
    [X, err_i] = line2d_iteration(Li, Lj, Omega, X);
    Xs(:,:, i) = X;

    disp("e:");
    disp(err_i);
    disp("X:");
    disp(X);
    es(i)=err_i;
  endfor;
endfunction;