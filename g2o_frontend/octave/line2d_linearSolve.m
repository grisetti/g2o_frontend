function [Xnew, err]= line2d_linearSolve(Li, Lj, Omega, X)
  x=zeros(6,1);
  x(1:2)=X(1,1:2)';
  x(3:4)=X(2,1:2)';
  x(5:6)=X(1:2,3);
	# computing only the rotational part
	H=zeros(4,4);
  b=zeros(4,1);
  err = 0;
  for k = 1:size(Li)(2)
    A=zeros(2,4);
    li = Li(:,k);
    lj = Lj(:,k);
    ni = li(1:2);
    #di = Li(3,k);
    nj = lj(1:2);
    #dj = Lj(3,k);
    A(1,1:2)=nj';
    A(2,3:4)=nj';
    #A(3,1:2)=x(5)*nj';
    #A(3,3:4)=x(6)*nj';
    #A(3,5:6))=(X(1:2,1:2)*nj)';
    ek = A*x(1:4)-li(1:2);
    H += A'*Omega(1:2,1:2)*A;
    b += A'*Omega(1:2,1:2)*ek;
    err += ek'*Omega(1:2,1:2)*ek;
  endfor
  x=H\-b;
  
  printf("partial (R) before rotation: %f\n", err);  
	
	# saving the rotational part of the X
  Xtemp = X;
  Xtemp(1,1:2)+=x(1:2)';
  Xtemp(2,1:2)+=x(3:4)';
  #Xtemp(1:2,3)+=x(5:6);
  
  #recondition the rotation
  [U,s,V] = svd(Xtemp(1:2,1:2));
  R=U*V';
  Xtemp(1:2,1:2)=R;
  t=Xtemp(1:2,3);
  H=zeros(2,2);
  b=zeros(2,1);

  Xnew=eye(3);
  Xnew(1:2,1:2)=R;
  Xnew(1:2,3)=t;

 #recompute the translation 
  err=0;
  for k = 1:size(Li)(2)
    li = Li(:,k);
    lj = Lj(:,k);   
    ni = li(1:2);
    di = li(3);
    nj = lj(1:2);
    dj = lj(3);
    A = nj'*R';
    ek = dj+A*t;
    H += A'*Omega(3,3)*A;
    b += A'*Omega(3,3)*ek;
    err += ek*Omega(3,3)*ek;
  endfor
  dt=H\-b;
  t+=dt;
  Xnew=eye(3);
  Xnew(1:2,1:2)=R;
  Xnew(1:2,3)=t;
  printf("partial (T) after rotation: %f\n", err);  


  err=0;
  for k = 1:size(Li)(2)
    li = Li(:,k);
    lj = line2d_remapCartesian(Xnew, Lj(:,k)); 
    ek=lj-li;
    ek(3)=0;
    err += ek' * Omega * ek;  
  endfor
  printf("after all: %f\n", err);  

endfunction
