#performs a sequence of iterations to find the transform that maps a set of points with normals onto another
# Pi:    a 6 x N matrix of the reference points
# Pj:    a 6 x N matrix of the points to move
# Omega: a the information matrix assumed to be the same for all points
# X:     a 4 x 4 matrix of the initial guess
# n:     num of iterations
#
# returns:
# Xs:    an array containing the resulting transform at each iteration
# es:    an array containing the evolution of the error

function [Xs, es] = pwn_solve(Pi, Pj, Omega, X, n)
  Xs = zeros(4, 4, n);
  es = zeros(1, n);
  for i = [1:n]
    [X, err_i] = pwn_iteration(Pi, Pj, Omega, X);
    Xs(:,:, i) = X;
    es(i)=err_i;
  endfor;
endfunction;