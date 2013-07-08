# compute error chi2 given two lines
# li: line 1 [nx ny rho]'
# lj: line 2 [nx ny rho]'
# T : the 3x3 isometry of the transformation

function er = computeError(li, lj, T)
  l_diff = zeros(size(l1));
  lj = line2d_remapCartesian(T, lj);
  l_diff = li-lj;
  er = l_diff(1,1)^2 + l_diff(2,1)^2 + l_diff(3,1)^2
end
