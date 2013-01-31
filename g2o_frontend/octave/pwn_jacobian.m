# computes the jacobian of a small perturbation applied around the actual transformation, in the domain of the minimal representation.
# lot of math here...
# the increment is parameterized as [dtx dty dtz tqx dqy dqz]
# X: the transformation
# p: the point
# J: the 6x6 jacobian of the point with normal
function J = pwn_jacobian(X, p)
  J=zeros(6,6);
  J(1:3, 1:3) = X(1:3, 1:3);
  J(1:3, 4:6) = - X(1:3, 1:3) * 2 * skew(p(1:3,1));
  J(4:6, 4:6) = - X(1:3, 1:3) * 2 * skew(p(4:6,1));
endfunction;