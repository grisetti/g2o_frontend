function [Xs, es] = calibrate_solve(x, Z, n)
  Xs = zeros(6, n);
  es = zeros(1, n);
  for i = [1:n]
    [x, err_i] = calibrate_iteration(x,Z);
    Xs(:,i) = x;
    es(i)=err_i;
  endfor;
endfunction;