# generate the lines (here i use the same generation procedure as for the points, since the
# lines in cartesian coords are mapped as points with a normal
# in this case i threat the normal as the direction




#  generate a transform
gtx = [10 50 60 .5 .2 .3]';
gtX = v2t(gtx);
printf("the ground truth transform is:\n");
disp(gtX);

#validate the transforms in the different representations
cl = line_cartesianNormalize(rand(6,1));
pl = line_cartesian2pluecker(cl);
e1 = cl - line_pluecker2cartesian(pl);
disp ("cart->pluecker->cart");
disp(norm(e1));

tcl = line_remapCartesian(gtX,cl);
tpl = line_remapPluecker(gtX,pl);
disp ("cart->t>pluecker - p->transform");
disp(line_pluecker2cartesian(tpl)');
disp(tcl');
disp ("pluecker->trnasform->cart - cart->transform");
disp(line_cartesian2pluecker(tcl)')
disp(tpl');

disp ("pluecker->trnasform->transform-1");
disp (pl - line_remapPluecker(inverse(gtX),tpl));

np = 100;
tscale = 100;
printf("Generating a sample set of %d lines with normal, distributed in a radius of %f meters\n\n", np, tscale);

Li = rand(6, np)- 0.5;
Lj = zeros(6,np);
Li(1:3, :) *= tscale;
for i = [1:size(Li)(2)]
  Li(4:6, i) /= norm(Li(4:6, i));
  Li(:,i) = line_cartesianNormalize(Li(:,i));
  Li(:,i) = line_cartesian2pluecker(Li(:,i));
  Lj(:,i) = line_remapPluecker(gtX, Li(:,i));
endfor;

X=eye(4);

Omega=eye(5);
Omega(1:3,1:3)*=1e-3;
Omega(4:5,4:5)*=1000;

[Xs, es] = line_solve(Li, Lj, Omega, X, 10);





