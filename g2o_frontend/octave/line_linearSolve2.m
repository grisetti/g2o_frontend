function [Xnew, err]= line_solve2(Li, Lj, Omega, X)
  x=zeros(12,1);
  x(1:3)=X(1,1:3)';
  x(4:6)=X(2,1:3)';
  x(7:9)=X(3,1:3)';
  x(10:12)=X(1:3,4);
  H=zeros(12,12);
  b=zeros(12,1);
  for k = 1:size(Li)(2)
    A=zeros(6,12);
    wi = Li(1:3,k);
    di = Li(4:6,k);
    wj = Lj(1:3,k);
    dj = Lj(4:6,k);
    A(1,1:3)=wj';
    A(2,4:6)=wj';
    A(3,7:9)=wj';
    A(1:3,10:12)=-skew(di);
    A(4:3,1:3)=dj';
    A(4:3,4:6)=dj';
    A(4:3,7:9)=dj';
    evk=li-A*x;
    H+=A'*Omega*A;
    b+=A'*evk;
  endfor
  x=H\b;
  x
endfunction