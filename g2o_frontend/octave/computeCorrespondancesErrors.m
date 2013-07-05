# compute a vector of errors given 2 sets of lines

function errors = computeCorrespondancesErrors(Li, Lj, T)
  errors = zeros(size(Li));
  for i = [1:size(Li)(2)]
        errors(:,i) = computeError(Li(:,i), Lj(:,i), T);
  endfor;
end
