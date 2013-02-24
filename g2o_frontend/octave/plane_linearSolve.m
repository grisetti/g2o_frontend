function [Xnew, err, Hk]= plane_linearSolve(Pi, Pj, Omega, X)
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
    printf("ROTATIONAL JACOBIAN OF PLANE %d \n",i);
    printf("Planes are \n");
    pi
    pj
    A
    
    #A(4,1:3)=x(10)*nj';
    #A(4,4:6)=x(11)*nj';
    #A(4,7:9)=x(12)*nj';
    #A(4,10:12)=(X(1:3,1:3)*nj)';
    printf("#######################\n");
    x
    ek=A*x(1:9)-pi(1:3);
    printf("ROTATIONAL HESSIAN OF PLANE %d \n",k);
    H+=A'*Omega(1:3,1:3)*A
    printf("ROTATIONAL b OF PLANE %d \n",k);
    b+=A'*Omega(1:3,1:3)*ek
    err += ek'*Omega(1:3,1:3)*ek;
  endfor

  printf("Solving the linear system:\n");
  printf("---------->H:\n");
  H
  printf("---------->b:\n");
  b
  x=H\-b;
  printf("---------->:\n");
  x  
  Hk=H;
 
  printf("---------->X\n")
  X
  Xtemp = X;
  Xtemp(1,1:3)+=x(1:3)';
  Xtemp(2,1:3)+=x(4:6)';
  Xtemp(3,1:3)+=x(7:9)';

  printf("---------->XTEMP\n")
  Xtemp
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
  printf("łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł\n");
  printf("łłłłłłłłłłł Ringraziamo Cthulhu la parte rotazionale è finitałłłłłłłłłłł\n");
  printf("łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł łłłłłłłłłłł\n");
  printf("X dopo il ricondizionamento appare come:\n");
  Xnew

 #recompute the translation 
  err=0;
  for k = 1:size(Pi)(2)
    printf("TRANSLATION JACOBIAN OF PLANE %d \n",k);
    printf("Planes are \n");
    pi = Pi(:,k)
    pj = Pj(:,k)
   
    ni = pi(1:3);
    di = pi(4);
    nj = pj(1:3);
    dj = pj(4);
    printf("TRANSLATION JACOBIAN IS\n");
    A=-nj'*R'
    ek=dj+A*t-di;
    printf("TRANSLATION HESSIAN IS\n");
    H+=A'*Omega(4,4)*A
    printf("TRANSLATION B IS\n");
    b+= (A'*Omega(4,4)*ek)
    err += ek*Omega(4,4)*ek;
  endfor
  printf("Solving the linear system, take two:\n");
  printf("---------->H:\n");
  H
  printf("---------->b:\n");
  b
  dt=H\-b;
  printf("---------->dt:\n");
  dt

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
    err += ek' * Omega * ek;  
  endfor
  printf("after all: %f\n", err);  

endfunction
