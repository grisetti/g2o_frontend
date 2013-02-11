function [Xnew, err]= plane_linearSolve(Pi, Pj, Omega, X)
  x=zeros(12,1);
  x(1:3)=X(1,1:3)';
  x(4:6)=X(2,1:3)';
  x(7:9)=X(3,1:3)';
  x(10:12)=X(1:3,4);
  H=zeros(9,9);
  b=zeros(9,1);
  err = 0;
  for k = 1:size(Pi)(2)
    A=zeros(3,9);
    pi = Pi(:,k);
    pj = Pj(:,k);
    ni = pi(1:3);
    #di = Pi(4,k);
    nj = pj(1:3);
    #dj = Pj(4,k);
    A(1,1:3)=nj';
    A(2,4:6)=nj';
    A(3,7:9)=nj';
    #A(4,1:3)=x(10)*nj';
    #A(4,4:6)=x(11)*nj';
    #A(4,7:9)=x(12)*nj';
    #A(4,10:12)=(X(1:3,1:3)*nj)';
    ek=A*x(1:9)-pi(1:3);
    H+=A'*Omega(1:3,1:3)*A;
    b+=A'*Omega(1:3,1:3)*ek;
    err += ek'*Omega(1:3,1:3)*ek;
  endfor
  x=H\-b;

  
  printf("partial (R) before rotation: %f\n", err);  

  Xtemp = X;
  Xtemp(1,1:3)+=x(1:3)';
  Xtemp(2,1:3)+=x(4:6)';
  Xtemp(3,1:3)+=x(7:9)';
  #Xtemp(1:3,4)+=x(10:12);
  
  #recondition the rotation
  [U,s,V] = svd(Xtemp(1:3,1:3));
  R=U*V';
  Xtemp(1:3,1:3)=R;
  t=Xtemp(1:3,4);
  H=zeros(3,3);
  b=zeros(3,1);

  Xnew=eye(4);
  Xnew(1:3,1:3)=R;
  Xnew(1:3,4)=t;

 #recompute the translation 
  err=0;
  for k = 1:size(Pi)(2)
    pi = Pi(:,k);
    pj = Pj(:,k);   
    ni = pi(1:3);
    di = pi(4);
    nj = pj(1:3);
    dj = pj(4);
    A=-nj'*R';
    ek=dj+A*t;
    H+=A'*Omega(4,4)*A;
    b+= (A'*Omega(4,4)*ek);
    err += ek*Omega(4,4)*ek;
  endfor
  dt=H\-b;
  t+=dt;
  Xnew=eye(4);
  Xnew(1:3,1:3)=R;
  Xnew(1:3,4)=t;
  printf("partial (T) after rotation: %f\n", err);  


  err=0;
  for k = 1:size(Pi)(2)
    pi = Pi(:,k);
    pj = plane_remapCartesian(Xnew, Pj(:,k)); 
    ek=pj-pi;
    ek(4)=0;
    err += ek' * Omega * ek;  
  endfor
  printf("after all: %f\n", err);  

endfunction
