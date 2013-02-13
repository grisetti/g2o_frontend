# generate the lines (here i use the same generation procedure as for the points, since the
# lines in cartesian coords are mapped as points with a normal
# in this case i threat the normal as the direction


#  generate a transform
gtx = [2 5 .3]';
gtX = v2t_2d(gtx);
printf("the ground truth transform is:\n");
disp(gtX);

np = 5;
tscale = 100;
printf("Generating a sample set of %d lines with normal, distributed in a radius of %f meters\n\n", np, tscale);

Li = rand(3, np)- 0.5;
Lj = zeros(3,np);
Li(1:2, :) *= tscale;
for i = [1:size(Li)(2)]
  Li(1:2, i) /= norm(Li(1:2, i));
  Lj(:,i) = line2d_remapCartesian(gtX, Li(:,i));
endfor;

X=eye(3);

Omega=eye(3);
Omega(1:2,1:2)*=1000;
Omega(3,3)*=1000;

#  [Xs, es] = line2d_solve(Li, Lj, Omega, X, 10);
[Xs, es] = line2d_linearSolve(Li, Lj, Omega, X);




