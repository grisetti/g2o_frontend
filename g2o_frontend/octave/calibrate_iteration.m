#computes the solution for one iteration of least squares to determine the position of the sensor
# given a set of mobile robot movements and a set of sensor movements

# Xnew: the new transformation
# err: the chi2 at the previous iteration

function [xnew, err]= calibrate_iteration(x, Z)
  b = zeros(6,1);
  H = zeros(6,6);
  Omega=eye(6);
  Omega(4:6,4:6)*=10000;
  err = 0;
  for i = [1:size(Z)(1)]
    z = Z(i,:)';
    ei = calibrate_error(x,z);
    Ji = calibrate_error_jacobian(x,z);
    bi = Ji' * Omega *  ei;
    Hi = Ji' * Omega * Ji;
    b += bi ;
    H += Hi ;
    err += ei' * Omega * ei;
  endfor
  H += 100*eye(6);
  dx = - H\b;
  dX = v2t(dx);
  X=v2t(x);
  xnew  = t2v(X*dX);
endfunction