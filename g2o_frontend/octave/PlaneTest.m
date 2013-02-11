# generate the lines (here i use the same generation procedure as for the points, since the
# lines in cartesian coords are mapped as points with a normal
# in this case i threat the normal as the direction


#  generate a transform
gtx = [1 5 6 .5 .2 .3]';
#gtx = [0 0 0 .0 .0 .0]';
gtX = v2t(gtx);
printf("the ground truth transform is:\n");
disp(gtX);

np = 5;
tscale = 100;
printf("Generating a sample set of %d lines with normal, distributed in a radius of %f meters\n\n", np, tscale);

Li = rand(4, np)- 0.5;
Lj = zeros(4,np);
Li(1:3, :) *= tscale;
for i = [1:size(Li)(2)]
  Li(1:3, i) /= norm(Li(1:3, i));
  Lj(:,i) = plane_remapCartesian(gtX, Li(:,i));
endfor;

X=eye(4);

Omega=eye(4);
Omega(1:3,1:3)*=1000;
Omega(4,4)*=1000;

#[Xs, es] = plane_solve(Li, Lj, Omega, X, 10);
[Xs, es] = plane_linearSolve(Li, Lj, Omega, X);



