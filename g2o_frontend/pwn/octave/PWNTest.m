# generate the points

np = 100;
tscale = 100;
printf("Generating a sample set of %d points with normal, distributed in a radius of %f meters\n\n", np, tscale);

Pi = rand(6, np)- 0.5;
Pi(1:3, :) *= tscale;
for i = [1:size(Pi)(2)]
  Pi(4:6, i) /= norm(Pi(4:6, i));
endfor;

#  generate a transform
gtx = [100 200 300 .5 .5 .5]';
gtX = v2t(gtx);
printf("the ground truth transform is:\n");
disp(gtX);



#generate a set of noised points with different levels
nLevel = [0 0.001  0.01 0.1 0.5];
Pjn = zeros(6, np, size(nLevel)(2));
Pjn(:, :, 1) = pwn_remapPoints(Pi, gtX);

Omegas = zeros(6,6,nLevel);
Omegas(:,:,1) = eye(6);
#Omegas(1:3, 1:3,1) *= 10;
Omegas(4:6, 4:6,1) *= 100;

printf("corrupting the measurements with noise levels\n");
printf ("level 1\n, omega = ");
disp(Omegas(:,:,1));
     
for i = [2:size(nLevel)(2)]
     Omegas(:,:,i) = Omegas(:,:,1)*1./sqrt(nLevel(i));
     Pjn(:, :, i) = pwn_addNoise(Pjn(:, :, 1), Omegas(:,:,i));
     printf ("level %d\n, omega = ", i);
     disp(Omegas(:,:,i));
endfor



# #run the test on all data
iterations = 40;
errors = zeros(iterations, nLevel);
results = zeros(4,4,iterations, nLevel);
printf("Running tests \n");
printf("\t iterations:%d", iterations);

for i = [1:size(nLevel)(2)]
  printf("\t optimizing for noise level %d\n", i);
  [results(:,:,:,i), errors(:,i)] = pwn_solve(Pi, Pjn(:, :, i), Omegas(:,:,i), eye(4), iterations);
  printf("\t initial error=%d\n", errors(1,i));
  printf("\t final   error=%d\n", errors(iterations,i));
  printf("\t final   transform\n");
  disp(results(:,:,iterations, i));
  printf("\t transform error\n");
  disp(t2v(results(:,:,iterations, i)*gtX));
endfor


#results (X,X, iteration, noise_level) 
# is a table containing the partial reuslts of the iterations
# first two values are the partial result
# third index is the iteration number
# fourth index is the noise level

#errors (e, iteration, noise_level) 
# chi2 with varying iteration and noise level







