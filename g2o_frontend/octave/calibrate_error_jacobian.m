# compute error of the parameters, given a guess X

function J = calibrate_error_jacobian(x, z)
  J=zeros(6,6);
  eps=1e-3;
  ieps = .5/eps;
  for i=[1:6]
    xup=x;
    xdwn=x;
   xup(i)+=eps;
   xdwn(i)-=eps;
   J(:,i)=(calibrate_error(xup,z) - calibrate_error(xdwn,z));
  end
  J*=ieps;
end
