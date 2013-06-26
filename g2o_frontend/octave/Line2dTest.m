# generate the lines (here i use the same generation procedure as for the points, since the
# lines in cartesian coords are mapped as points with a normal
# in this case i threat the normal as the direction


#  generate a transform
gtx = [2 5 .3]';
printf("ground truth vector is:\n");
disp(gtx);
gtX = v2t_2d(gtx);
printf("the ground truth transform is:\n");
disp(gtX);

np = 5;
#tscale = 100;
#printf("Generating a sample set of %d lines with normal, distributed in a radius of %f meters\n\n", np, tscale);
#Li = rand(3, np)- 0.5;
#Lj = zeros(3,np);
#Li(1:2, :) *= tscale;

Li = zeros(3,np);
Li(1:3, 1) = [cos(-1), sin(-1), 67.6643]';
Li(1:3, 2) = [cos(-0.0255656), sin(-0.0255656), 73.3178]';
Li(1:3, 3) = [cos(0.112189), sin(0.112189), 51.8915]';
Li(1:3, 4) = [cos(0.0631137), sin(0.0631137), 131.043]';
Li(1:3, 5) = [cos(-0.846451), sin(-0.846451), 61.8352]';
disp("Li:");
disp(Li);
for i = [1:size(Li)(2)]
  #Li(1:2, i) /= norm(Li(1:2, i));
 Lj(:,i) = line2d_remapCartesian(gtX, Li(:,i));
endfor;

disp("Lj:");
disp(Lj);

X=eye(3);

Omega=eye(3);
Omega(1:2,1:2)*=1000;
Omega(3,3)*=1000;

# choose line2d_solve if you want a non linear solving
[Xs, es] = line2d_solve(Li, Lj, Omega, X, 10);
# [Xs, es] = line2d_linearSolve(Li, Lj, Omega, X);
disp("- final trasform ");
Xs_final = zeros(3,3);
Xs_final = Xs(:,:,10);
disp(Xs_final);
disp("Transform error vector:");
disp(t2v_2d(gtX*Xs_final));
# disp("Transform error:");
# disp(gtX*Xs_final);


#debug: controllo se la trasf trovata * Lj =  Li (è l'inversa)
TLj = zeros(3, np);
for i = [1:size(Lj)(2)]
  #Li(1:2, i) /= norm(Li(1:2, i));
 TLj(:,i) = line2d_remapCartesian(Xs_final, Lj(:,i));
endfor;
TLj
