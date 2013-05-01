Li=load("L1.dat");
Lk=load("L2.dat");
Li=Li';
Lk=Lk';
printf("Li generated\n");
printf("Lk generated\n");

gtx = [1 5 6 .5 .2 .3]';
gtX = v2t(gtx);
np = 3;

for i = [1:size(Li)(2)]
  #Li(1:3, i) /= norm(Li(1:3, i));
  Lj(:,i) = plane_remapCartesian(gtX, Li(:,i));
endfor;
printf("Generating Lj\n");

X=eye(4);

Omega=eye(4);
Omega(1:3,1:3)*=1;
Omega(4,4)*=1;

[Xs, es, H] = plane_linearSolve(Li, Lj, Omega, X);
#[Xk, ek, Hk] = plane_linearSolve(Li, Lk, Omega, X);
