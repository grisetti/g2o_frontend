function [Xnew, err]= line_linearSolve(Li, Lj, Omega, X)
  x=zeros(12,1);
  x(1:3)=X(1,1:3)';
  x(4:6)=X(2,1:3)';
  x(7:9)=X(3,1:3)';
  x(10:12)=X(1:3,4);
  H=zeros(12,12);
  b=zeros(12,1);
  err = 0;
  for k = 1:size(Li)(2)
    A=zeros(6,12);
    li = Li(:,k);
    lj = Lj(:,k);
    wi = Li(1:3,k);
    di = Li(4:6,k);
    wj = Lj(1:3,k);
    dj = Lj(4:6,k);
    A(1,1:3)=wj';
    A(2,4:6)=wj';
    A(3,7:9)=wj';
    A(1:3,10:12)=-skew(di);
    A(4,1:3)=dj';
    A(5,4:6)=dj';
    A(6,7:9)=dj';
    ek=li-A*x;
    H+=A'*Omega*A;
    b+=A'*Omega*ek;
    err += ek'*Omega*ek;
  endfor
  x=H\b;
  

  Xtemp = X;
  Xtemp(1,1:3)+=x(1:3)';
  Xtemp(2,1:3)+=x(4:6)';
  Xtemp(3,1:3)+=x(7:9)';
  Xtemp(1:3,4)+=x(10:12);
  
  #recondition the rotation
  [U,s,V] = svd(Xtemp(1:3,1:3));
  R=U*V';
  t=Xtemp(1:3,4);
  H=zeros(3,3);
  b=zeros(3,1);
  #recompute the translation 
  for k = 1:size(Li)(2)
    li = Li(:,k);
    lj = Lj(:,k);
    wi = li(1:3);
    wj = lj(1:3);
    dj = lj(4:6);
    A=-skew(R*dj);
    ek=wi-R*wj-A*t;
    H+=A'*Omega(1:3,1:3)*A;
    b+=A'*Omega(1:3,1:3)*ek;
  endfor
  dt=H\b;
  t+=dt;
  Xnew=eye(4);
  Xnew(1:3,1:3)=R;
  Xnew(1:3,4)=t;
endfunction
